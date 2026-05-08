#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_console.h"
#include "esp_check.h"
#include "argtable3/argtable3.h"
#include "linenoise/linenoise.h"
#include "canopen.h"
#include "canopen_client.h"
#include "canopen_console.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TCP_PORT 3344  // Puerto conforme a CANopen CiA 309

static const char *TAG = "cia309";
static TaskHandle_t usb_console_task_handle;
static TaskHandle_t tcp_console_task_handle;


/*
    CiA 309-3 ASCII console

    See https://canopennode.github.io/CANopenSocket/group__CO__CANopen__309__3__Syntax.html

    It is mostly conformant with CiA 309-3 except for negative numbers. You need to prepend --
    before negative numbers are accepted. E.g.

    w 0x606c 0 i32 -- -300
    
 */


#define N_ELEMS(x) (sizeof(x)/sizeof(x[0]))

// parsers, printers, type information for supported datatypes
// suport functions and variables for standard CiA 309-3 datatypes
#define CT(datatype, ctype, scan_sfx, print_sfx) \
    static void parse_##ctype(const char* str, void* buf)  { *(ctype*)buf = (ctype)strto##scan_sfx(str, NULL, 0); } \
    static void print_##ctype(void* buf) { printf("%" #print_sfx, *(ctype*)buf); }
#define CTA(datatype, ctype, scan_sfx, print_sfx) \
    static void parse_##datatype##_t(const char* str, void* buf)  { *(ctype*)buf = (ctype)strto##scan_sfx(str, NULL, 0); } \
    static void print_##datatype##_t(void* buf) { printf("%" #print_sfx, *(ctype*)buf); }
#define CTS(datatype, ctype) \
    static void parse_##ctype(const char* str, void* buf) { *(uint64_t*)buf = *(uint64_t*)str; } \
    static void print_##ctype(void* buf) { printf("%s", (char*)buf); }
CANOPEN_CONSOLE_TYPES(CT, CTA, CTS)


// individual console type descriptors
#define ENTRY_T_DEF(datatype, ctype, scan_sfx, print_sfx) \
    { #datatype, sizeof(ctype), parse_##ctype, print_##ctype },
#define ENTRY_A_DEF(datatype, ctype, scan_sfx, print_sfx) \
    { #datatype, sizeof(ctype), parse_##datatype##_t, print_##datatype##_t },
#define ENTRY_S_DEF(datatype, ctype) \
    { #datatype, 8, parse_##ctype, print_##ctype },
canopen_type_entry_t by_type[] = {
    CANOPEN_CONSOLE_TYPES(ENTRY_T_DEF, ENTRY_A_DEF, ENTRY_S_DEF)
};

#define R(o) ((o)->readable ? 'R' : '-')
#define W(o) ((o)->writable ? 'W' : '-')

/*
    We introduce the following non-standard syntax:

    write control_word 4    Write object control_word 
    read status_word        Read object status_word, type is already in the object dictionary
    help contr              Shows information on all objects startinng with 'contr'
 */

static const object_dictionary_entry_t* od = NULL;
static size_t od_len = 0;


// command context -------------------------------------------

// default_ctx is managed with set command
// ctx copies default values from default_ctx but current statement may override them 
console_context_t ctx;
static console_context_t default_ctx = {
    .sequence = 0,
    .net = 0,
    .node = 0,
    .sdo_timeout = 0,
    .sdo_block = false,
    .dump_msg = false,
};

// command argument tables --------------------------------- 

static struct {
    struct arg_int *index;
    struct arg_int *subindex;
    struct arg_str *datatype;
    struct arg_end *end;
} read_args;

static struct {
    struct arg_str *symbol;
    struct arg_str *datatype;
    struct arg_end *end;
} read_sym_args;

static struct {
    struct arg_int *index;
    struct arg_int *subindex;
    struct arg_str *datatype;
    struct arg_str *value;
    struct arg_end *end;
} write_args;

static struct {
    struct arg_str *symbol;
    struct arg_str *svalue;
    struct arg_end *end;
} write_sym_args;

static struct {
    struct arg_str *param;  // node|communication
    struct arg_end *end;
} reset_args;

static struct {
    struct arg_str *param;  // network|node|sdo_timeout|sdo_block|dump
    struct arg_str *value;  // valor a establecer
    struct arg_end *end;
} set_args;

static struct {
    struct arg_str *param;  // network|node|sdo_timeout|sdo_block|dump
    struct arg_end *end;
} get_args;

static struct {
    struct arg_str *topic;  // object|datatype|about
    struct arg_end *end;
} about_args;

// ----------------------------------------------------------

const char* trim_spaces(const char* line)
{
    while(isspace((int)*line)) ++line;
    return line;
}

void print_result_int(int value) {
    printf("[%d] %d\r\n", ctx.sequence, value);
}

void print_result_value(object_dictionary_entry_t* obj, object_value_t* value) {
    printf("[%d] ", ctx.sequence);
    obj->type->print(value);
    printf("\r\n");
}

void print_result_ok(void) {
    printf("[%d] OK\r\n", ctx.sequence);
}

void print_result_error(const char *msg) {
    printf("[%d] ERROR:%s\r\n", ctx.sequence, msg);
}

static canopen_type_entry_t* get_type_by_datatype(const char* datatype) {
    for (uint8_t i = 0; i < N_ELEMS(by_type); ++i) {
        if (0 == strcmp(datatype, by_type[i].datatype)) {
            return &by_type[i];
        }
    }
    return NULL;
}

object_dictionary_entry_t* get_dictionary_entry(const char* sym, const char* datatype)
{
    static object_dictionary_entry_t ret; // just one entry being read, no need to allocate
    ret.id = NULL;
    for (size_t i = 0; i < od_len; ++i) {
        if (0 == strcmp(sym, od[i].id)) {
            ret = od[i];
            break;
        }
    }

    if (ret.id == NULL) return NULL;
    if (datatype == NULL) return &ret;

    canopen_type_entry_t* type = get_type_by_datatype(datatype);
    ret.type = type ? type : &by_type[x32_type_info]; // default to int32_t if datatype not found

    return &ret;
}

static object_dictionary_entry_t* get_object_entry(uint16_t index, uint8_t subindex, const char* datatype)
{
    static object_dictionary_entry_t ret; // just one entry being read, no need to allocate
    ret.index = index;
    ret.subindex = subindex;
    canopen_type_entry_t* type = get_type_by_datatype(datatype);
    ret.type = type ? type : &by_type[x32_type_info]; // default to int32_t if datatype not found
    return &ret;
}

static void read_object(object_dictionary_entry_t* obj) 
{
    if (!obj->readable) {
        print_result_error("Object not readable");
        return;
    }
    object_value_t value;
    esp_err_t err = sdo_upload(0x600 + ctx.node, obj->index, obj->subindex, &value);
    if (err == ESP_OK) print_result_value(obj, &value);
    else print_result_error("Unsuccessful SDO upload");
}

static void write_object(object_dictionary_entry_t* obj, object_value_t* value) 
{ 
    if (!obj->writable) {
        print_result_error("Object not writable");
        return;
    }
    esp_err_t err = sdo_download(0x600 + ctx.node, obj->index, obj->subindex, value, obj->type->size);
    if (err == ESP_OK) print_result_ok();
    else print_result_error("Unsuccessful SDO download");
}

static int cmd_read_sym(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&read_sym_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, read_sym_args.end, argv[0]);
        return 1;
    }
    const char *sym = read_sym_args.symbol->sval[0];
    const char *datatype = read_sym_args.datatype->count > 0 ? read_sym_args.datatype->sval[0] : NULL;
    object_dictionary_entry_t* obj = get_dictionary_entry(sym, datatype);
    if (obj == NULL) print_result_error("Unknown object");
    else read_object(obj);
    return 0;
}

static int cmd_read(int argc, char **argv)
{
    char* arg_index = argv[1];
    if (argc > 1 && !isdigit((unsigned char)arg_index[0])) return cmd_read_sym(argc, argv);

    int nerrors = arg_parse(argc, argv, (void **)&read_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, read_args.end, argv[0]);
        return 1;
    }
    uint16_t index = *read_args.index->ival;
    uint8_t subindex = *read_args.subindex->ival;
    const char *datatype = read_args.datatype->count > 0 ? read_args.datatype->sval[0] : NULL;

    read_object(get_object_entry(index, subindex, datatype));
    return 0;
}


static int cmd_write_sym(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&write_sym_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, write_sym_args.end, argv[0]);
        return 1;
    }

    const char *sym = write_sym_args.symbol->sval[0];
    const char* svalue = write_sym_args.svalue->sval[0];

    object_dictionary_entry_t* obj = get_dictionary_entry(sym, NULL);
    if (obj == NULL) {
        print_result_error("Unknown object");
        return -1;
    }

    object_value_t value;
    obj->type->parse(svalue, &value);
    write_object(obj, &value);
    return 0;
}

static int cmd_write(int argc, char **argv)
{
    char* arg_index = argv[1];
    if (argc > 1 && !isdigit((unsigned char)arg_index[0])) return cmd_write_sym(argc, argv);

    int nerrors = arg_parse(argc, argv, (void **)&write_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, write_args.end, argv[0]);
        return 1;
    }
    uint16_t index = *write_args.index->ival;
    uint8_t subindex = *write_args.subindex->ival;
    const char *datatype = write_args.datatype->sval[0];
    const char *sval = write_args.value->sval[0];

    object_dictionary_entry_t* obj = get_object_entry(index, subindex, datatype);
    object_value_t value;
    obj->type->parse(sval, &value);
    write_object(obj, &value);
    return 0;
}

static int cmd_start(int argc, char **argv)
{
    nmt_start_remote_node(ctx.node);
    print_result_ok();
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    nmt_stop_remote_node(ctx.node);
    print_result_ok();
    return 0;
}

static int cmd_preoperational(int argc, char **argv)
{
    nmt_enter_preoperational(ctx.node);
    print_result_ok();
    return 0;
}

static int cmd_reset(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&reset_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, reset_args.end, argv[0]);
        print_result_error("invalid_args");
        return 1;
    }

    const char *param = reset_args.param->sval[0];

    if (strcasecmp(param, "node") == 0) {
        nmt_reset_node(ctx.node);
        print_result_ok();
    } 
    else if (strcasecmp(param, "comm") == 0 || strcasecmp(param, "communication") == 0) {
        nmt_reset_communication(ctx.node);
        print_result_ok();
    } 
    else {
        print_result_error("unknown_param");
        return 1;
    }
    return 0;
}

static int cmd_set(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&set_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_args.end, argv[0]);
        print_result_error("invalid_args");
        return 1;
    }

    const char *param = set_args.param->sval[0];
    const char *val_str = set_args.value->sval[0];

    if (strcasecmp(param, "network") == 0) {
        int val = strtol(val_str, NULL, 0);
        default_ctx.net = val;
        print_result_ok();
    }
    else if (strcasecmp(param, "node") == 0) {
        int val = strtol(val_str, NULL, 0);
        default_ctx.node = val;
        print_result_ok();
    }
    else if (strcasecmp(param, "sdo_timeout") == 0) {
        int val = strtol(val_str, NULL, 0);
        canopen_max_delay_ms(val);
        default_ctx.sdo_timeout = canopen_get_max_delay_ms();
        print_result_ok();
    }
    else if (strcasecmp(param, "sdo_block") == 0) {
        if (strcmp(val_str, "1") == 0 || strcasecmp(val_str, "true") == 0) {
            default_ctx.sdo_block = true;
        }
        else {
            default_ctx.sdo_block = false;
        }
        print_result_ok();
    }
    else if (strcasecmp(param, "dump") == 0) {
        if (strcmp(val_str, "1") == 0 || strcasecmp(val_str, "true") == 0) {
            canopen_dump_enabled(true);
            default_ctx.dump_msg = true;
        }
        else {
            canopen_dump_enabled(false);
            default_ctx.dump_msg = false;
        }
        print_result_ok();
    }
    else {
        print_result_error("unknown_param");
        return 1;
    }

    return 0;
}

static int cmd_get(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&get_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_args.end, argv[0]);
        print_result_error("invalid_args");
        return 1;
    }

    const char *param = get_args.param->sval[0];
    if (strcasecmp(param, "network") == 0) {
        print_result_int(ctx.net);
    }
    else if (strcasecmp(param, "node") == 0) {
        print_result_int(ctx.node);
    }
    else if (strcasecmp(param, "sdo_timeout") == 0) {
        print_result_int(canopen_get_max_delay_ms());
    }
    else if (strcasecmp(param, "sdo_block") == 0) {
        print_result_int(ctx.sdo_block);
    }
    else if (strcasecmp(param, "dump") == 0) {
        print_result_int(canopen_is_dump_enabled());
    }
    else {
        print_result_error("unknown_param");
        return 1;
    }

    return 0;
}

static int cmd_exit(int argc, char **argv)
{
    canopen_request_shutdown();
    // not reached
    return 0;
}

#define NN "  [[net] node] "
#define N  "  [net] "

static int cmd_about(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&about_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, about_args.end, argv[0]);
        return 1;
    }

    if (about_args.topic->count == 0) {
        printf( "EPOS-CAN brought to you by:\r\n"
                "  Mantis Research Group\r\n"
                "  Institute of Applied Research for the Aeronautical Industry\r\n"
                "  University of Castilla-La Mancha\r\n"
                "  45005 Toledo, Spain\r\n"
                "  mantis@on.uclm.es\r\n"
                "  https://uclm-mantis.github.io/\r\n");
        printf("Available commands:\r\n"
                NN "r[ead] <index> <subindex> [<datatype>]\r\n"
                NN "r[ead] <symbol> [<datatype>]\r\n"
                NN "w[rite] <index> <subindex> <datatype> <value>\r\n"
                NN "w[rite] <symbol> <value>\r\n"
                NN "start\r\n"
                NN "stop\r\n"
                NN "preop[erational]\r\n"
                NN "reset node|comm[unication]\r\n"
                N "set network|node|sdo_timeout|sdo_block|dump <value>\r\n"
                "  about [datatype|object|<symbol prefix>]\r\n");
    } else {
        const char *topic = about_args.topic->sval[0];
        if (strcasecmp(topic, "about") == 0) {
        } else if (strcasecmp(topic, "datatype") == 0) {
            printf( "Available datatypes:\r\n"
                    "  b                  # boolean.\r\n"
                    "  i8, i16, i32, i64  # signed integers.\r\n"
                    "  u8, u16, u32, u64  # unsigned integers.\r\n"
                    "  x8, x16, x32, x64  # unsigned integers in hex.\r\n"
                    "  vs                 # visible string.\r\n");
        } else if (strcasecmp(topic, "object") == 0) {
            printf( "Available objects:\r\n");
            for (size_t i = 0; i < od_len; ++i) {
                const object_dictionary_entry_t* o = &od[i];
                printf("  %04x %02x %-3s %c%c %s\r\n", o->index, o->subindex, o->type->datatype, R(o), W(o), o->id);
            }
        } else {
            printf( "Objects starting with %s:\r\n", topic);
            for (size_t i = 0; i < od_len; ++i) {
                const object_dictionary_entry_t* o = &od[i];
                if (0 == strncmp(o->id, topic, strlen(topic)))
                    printf("  %04x %02x %-3s %c%c %s\r\n", o->index, o->subindex, o->type->datatype, R(o), W(o), o->id);
            }
            return 1;
        }
    }

    return 0;
}

void canopen_console_register_commands() 
{
    read_args.index = arg_int1(NULL, NULL, "<index>", "Object index (16bit)");
    read_args.subindex = arg_int1(NULL, NULL, "<subindex>", "Object subindex (8bit)");
    read_args.datatype = arg_str0(NULL, NULL, "<datatype>", "Object data type code (see help datatype)");
    read_args.end = arg_end(1);

    read_sym_args.symbol = arg_str1(NULL, NULL, "<symbol>", "Symbolic object identifier (see help object)");
    read_sym_args.datatype = arg_str0(NULL, NULL, "<datatype>", "Object data type code (see help datatype)");
    read_sym_args.end = arg_end(1);

    write_args.index = arg_int1(NULL, NULL, "<index>", "Object index (16bit)");
    write_args.subindex = arg_int1(NULL, NULL, "<subindex>", "Object subindex (8bit)");
    write_args.datatype = arg_str1(NULL, NULL, "<datatype>", "Object data type code (see help datatype)");
    write_args.value = arg_str1(NULL, NULL, "<value>", "Object value");
    write_args.end = arg_end(1);

    write_sym_args.symbol = arg_str1(NULL, NULL, "<symbol>", "Symbolic object identifier (see help object)");
    write_sym_args.svalue = arg_str1(NULL, NULL, "<value>", "Object value as string");
    write_sym_args.end = arg_end(1);

    reset_args.param = arg_str1(NULL, NULL, "<node|communication>", "Entity to be reset");
    reset_args.end = arg_end(1);

    set_args.param = arg_str1(NULL, NULL, "<network|node|sdo_timeout|sdo_block|dump>", "Parameter to set");
    set_args.value = arg_str1(NULL, NULL, "<value>", "Parameter value");
    set_args.end = arg_end(1);

    get_args.param = arg_str1(NULL, NULL, "<network|node|sdo_timeout|sdo_block|dump>", "Parameter to get");
    get_args.end = arg_end(1);

    about_args.topic = arg_str0(NULL, NULL, "<object|datatype|about|symbol>", "Help topic");
    about_args.end = arg_end(1);

    const esp_console_cmd_t cmds[] = {
        { .command = "start", .help = "NMT Start node", .hint = NULL, .func = &cmd_start, .argtable = NULL },
        { .command = "stop", .help = "NMT Stop node", .hint = NULL, .func = &cmd_stop, .argtable = NULL },
        { .command = "preop", .help = "NMT enter preoperational state", .hint = NULL, .func = &cmd_preoperational, .argtable = NULL },
        { .command = "preoperational", .help = "NMT enter preoperational state", .hint = NULL, .func = &cmd_preoperational, .argtable = NULL },
        { .command = "reset", .help = "NMT reset node|communication", .hint = NULL, .func = &cmd_reset, .argtable = &reset_args },
        { .command = "read", .help = "SDO upload object", .hint = NULL, .func = &cmd_read, .argtable = &read_args },
        { .command = "r", .help = "SDO upload object", .hint = NULL, .func = &cmd_read, .argtable = &read_args },
        { .command = "write", .help = "SDO download object", .hint = NULL, .func = &cmd_write, .argtable = &write_args },
        { .command = "w", .help = "SDO download object", .hint = NULL, .func = &cmd_write, .argtable = &write_args },
        { .command = "set", .help = "Set default parameter", .hint = NULL, .func = &cmd_set, .argtable = &set_args },
        { .command = "get", .help = "Get default parameter", .hint = NULL, .func = &cmd_get, .argtable = &get_args },
        { .command = "exit", .help = "End CAN tasks", .hint = NULL, .func = &cmd_exit, .argtable = NULL },
        { .command = "about", .help = "Information on some topics", .hint = NULL, .func = &cmd_about, .argtable = &about_args },
    };

    for (uint8_t i = 0; i< N_ELEMS(cmds); ++i) {
        ESP_ERROR_CHECK(esp_console_cmd_register(cmds+i));
    }
    ESP_ERROR_CHECK(esp_console_register_help_command());
}

static void canopen_console_run(char* line)
{
    // inherits default context
    ctx = default_ctx;

    // 1) "[<seq>]"
    line = (char*) trim_spaces(line);
    if (line[0] == '[') {
        const char *end = strchr(line, ']');
        if (end) {
            ctx.sequence = strtol(line + 1, NULL, 0);
            line = (char*) trim_spaces(end + 1);
        }
    }

    // 2) [[net] node]
    if (isdigit((unsigned char)line[0])) {
        char *end1;
        long first = strtol(line, &end1, 0);
        end1 = (char*)trim_spaces(end1);

        if (isdigit((unsigned char)end1[0])) {
            char *end2;
            long second = strtol(end1, &end2, 0);
            ctx.net = (int)first;
            ctx.node = (int)second;
            line = (char*)trim_spaces(end2);
        } else if (strncmp(end1, "set", 3) == 0 &&
                   (end1[3] == '\0' || isspace((unsigned char)end1[3]))) {
            // En "set", el prefijo numérico es solo [net]
            ctx.net = (int)first;
            line = end1;
        } else {
            // Un único número: [node]
            ctx.node = (int)first;
            line = end1;
        }
    }

    int ret;
    esp_console_run(line, &ret);
}

static void suggest_completions(const char *cmd, const char* sym, linenoiseCompletions *lc, bool read, bool write) 
{
    size_t len = strlen(sym);
    if (len == 0) return;
    for (size_t i = 0; i < od_len; i++) {
        bool maybe = (read && od[i].readable) || (write && od[i].writable);
        if (maybe && strncmp(sym, od[i].id, len) == 0) {
            char line[256];
            sprintf(line, "%s%s", cmd, od[i].id);
            linenoiseAddCompletion(lc, line);
        }
    }
}

static void completion_callback(const char *buf, linenoiseCompletions *lc) 
{
    esp_console_get_completion(buf, lc);

    static struct {
        const char* start;
        bool read, write;
    } cmd[] = {
        { "read ", true, false },
        { "r ", true, false },
        {"write ", false, true }, 
        {"w ", false, true}, 
        {"about ", true, true }
    };

    for (size_t i=0; i < N_ELEMS(cmd); ++i) {
        if (strncmp(buf, cmd[i].start, strlen(cmd[i].start)) == 0) {
            const char *partial_sym = trim_spaces(buf + strlen(cmd[i].start)); 
            suggest_completions(cmd[i].start, partial_sym, lc, cmd[i].read, cmd[i].write);
        }
    }
}

void canopen_console_task(void *arg);
void tcp_console_task(void *arg);

void canopen_console_init(const canopen_console_cfg_t* cfg)
{
    // Si cfg es NULL, aplicar configuración por defecto.
    canopen_console_cfg_t default_cfg = CANOPEN_CONSOLE_DEFAULT();
    if (cfg == NULL) {
        cfg = &default_cfg;
    }

    // Inicializar valores de contexto con valores reales del canopen core.
    default_ctx.sdo_timeout = canopen_get_max_delay_ms();
    default_ctx.dump_msg = canopen_is_dump_enabled();

    fflush(stdout);
    fsync(fileno(stdout));

    // ---- CONFIGURAR DISPOSITIVO DE CONSOLA SOBRE USB SERIAL/JTAG ----
    // Equivalente a la rama USB-SJTAG del ejemplo console/advanced

    // Fin de línea que llega del monitor (idf.py monitor manda CR al pulsar Enter)
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    // Cómo queremos que se envíen los \n
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    // Modo bloqueante en stdin/stdout (importante para linenoise)
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin),  F_SETFL, 0);

    // Instalar driver USB-SJTAG
    usb_serial_jtag_driver_config_t jtag_cfg = {
        .tx_buffer_size = cfg->tx_buffer_size,   // o un valor fijo, p.ej. 256
        .rx_buffer_size = cfg->rx_buffer_size,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&jtag_cfg));

    // Decir al VFS que use el driver USB-SJTAG para stdin/stdout/stderr
    usb_serial_jtag_vfs_use_driver();

    // Importante para que no haya buffering en stdin
    setvbuf(stdin, NULL, _IONBF, 0);

    esp_console_config_t console_config = {
        .max_cmdline_args   = 8,
        .max_cmdline_length = 256,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    linenoiseSetMultiLine(0);
    linenoiseSetCompletionCallback(&completion_callback);
    linenoiseHistorySetMaxLen(100);
    linenoiseSetMaxLineLen(console_config.max_cmdline_length);
    linenoiseAllowEmpty(false);

    od = cfg->object_dictionary;
    od_len = cfg->object_dictionary_entries;
    ESP_LOGI(TAG, "Console initialized with %d object dictionary entries", (int)od_len);

    if (cfg->dumb_mode) {
        ESP_LOGI(TAG, "Dumb mode enabled. Line editing and history features are disabled, but ANSI escape codes will be printed (e.g. colors).");
        linenoiseSetDumbMode(1);
    }

    if (cfg->enable_tcp_console) {
        xTaskCreatePinnedToCore(tcp_console_task, "tcp_console", 4096, NULL, 8, &tcp_console_task_handle, tskNO_AFFINITY);
    }

    if (cfg->enable_usb_console) {
        xTaskCreatePinnedToCore(canopen_console_task, "usb_console", 4096, NULL, 8, &usb_console_task_handle, tskNO_AFFINITY);
    }
}


void canopen_console_task(void *arg)
{
    for(;;) {
        char* line = linenoise(CONFIG_EPOS_CONSOLE_PROMPT);
        if (line == NULL) continue;
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            canopen_console_run(line);
        }
        linenoiseFree(line);
    }
    vTaskDelete(NULL);
}


void tcp_console_task(void *arg)
{
    int listen_sock = -1, client_sock = -1;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char rx_buffer[128];

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Error socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Error bind: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // Escuchar conexiones entrantes (se admiten 2 conexiones en cola)
    if (listen(listen_sock, 2) < 0) {
        ESP_LOGE(TAG, "Error listen: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP server listening in port %d", TCP_PORT);

    // Esperar a la conexión de un cliente
    client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
    if (client_sock < 0) {
        ESP_LOGE(TAG, "Error accept: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Client connected");

    for (;;) {
        send(client_sock, CONFIG_EPOS_CONSOLE_PROMPT, strlen(CONFIG_EPOS_CONSOLE_PROMPT), 0);
    
        int len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error recv: %d", errno);
            break;
        } else if (len == 0) {
            ESP_LOGI(TAG, "Client closed connection");
            break;
        }
        // Terminar la cadena recibida
        rx_buffer[len] = '\0';

        // Buscar y eliminar el salto de línea (si existe)
        char *newline = strchr(rx_buffer, '\n');
        if (newline) {
            *newline = '\0';
            if (strlen(rx_buffer) > 0) {
                canopen_console_run(rx_buffer);
            }
        }
    }

    close(client_sock);
    close(listen_sock);
    vTaskDelete(NULL);
}
