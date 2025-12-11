#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_console.h"
#include "esp_check.h"
#include "argtable3/argtable3.h"
#include "linenoise/linenoise.h"
#include "epos.h"
#include "epos_console.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cia309";

/*
    CiA 309-3 ASCII console

    See https://canopennode.github.io/CANopenSocket/group__CO__CANopen__309__3__Syntax.html

    It is mostly conformant with CiA 309-3 except for negative numbers. You need to prepend --
    before negative numbers are accepted. E.g.

    w 0x606c 0 i32 -- -300
    
 */


#define TCP_PORT 3344  // Puerto conforme a CANopen CiA 309
#define N_ELEMS(x) (sizeof(x)/sizeof(x[0]))

static void parse_int64_t(const char* str, void* buf)  { *(int64_t*)buf = strtoll(str, NULL, 0); }
static void parse_int32_t(const char* str, void* buf)  { *(int32_t*)buf = strtol(str, NULL, 0); }
#define parse_int16_t parse_int32_t
#define parse_int8_t parse_int32_t
static void parse_uint64_t(const char* str, void* buf) { *(uint64_t*)buf = strtoull(str, NULL, 0); }
static void parse_uint32_t(const char* str, void* buf) { *(uint32_t*)buf = strtoul(str, NULL, 0); }
#define parse_uint16_t parse_uint32_t
#define parse_uint8_t parse_uint32_t
static void parse_string4_t(const char* str, void* buf) { *(uint32_t*)buf = *(uint32_t*)str; }
static void parse_string8_t(const char* str, void* buf) { *(uint64_t*)buf = *(uint64_t*)str; }

static void print_int64_t(void* buf)   { printf("%lld", *(int64_t*)buf); }
static void print_x64_t(void* buf)     { printf("%016llx", *(int64_t*)buf); }
static void print_int32_t(void* buf)   { printf("%ld", *(int32_t*)buf); }
static void print_x32_t(void* buf)     { printf("%08lx", *(int32_t*)buf); }
static void print_int16_t(void* buf)   { printf("%d", *(int16_t*)buf); }
static void print_x16_t(void* buf)     { printf("%04x", *(int16_t*)buf); }
static void print_int8_t(void* buf)    { printf("%d", *(int8_t*)buf); }
static void print_x8_t(void* buf)      { printf("%02x", *(int8_t*)buf); }
static void print_uint64_t(void* buf)  { printf("%lld", *(int64_t*)buf); }
static void print_uint32_t(void* buf)  { printf("%ld", *(int32_t*)buf); }
static void print_uint16_t(void* buf)  { printf("%d", *(int16_t*)buf); }
static void print_uint8_t(void* buf)   { printf("%d", *(int8_t*)buf); }
static void print_string4_t(void* buf) { printf("%s", (char*)buf); }
static void print_string8_t(void* buf) { printf("%s", (char*)buf); }

#define CiA309type(type) type##_abbr
#define bool_abbr      "u64"
#define int8_t_abbr    "i8"
#define uint8_t_abbr   "u8"
#define int16_t_abbr   "i16"
#define uint16_t_abbr  "u16"
#define int32_t_abbr   "i32"
#define uint32_t_abbr  "u32"
#define int64_t_abbr   "i64"
#define uint64_t_abbr  "u64"
#define string8_t_abbr "vs"

#define R(o) ((o)->print ? 'R' : '-')
#define W(o) ((o)->parse ? 'W' : '-')

/*
    The object dictionary is only needed for easier interactive console. It is not needed for programmatic behaviours.
    We introduce the following non-standard syntax:

    write control_word 4    Write object control_word 
    read status_word        Read object status_word, type is already in the object dictionary
    help contr              Shows information on all objects startinng with 'contr'
 */

object_dictionary_entry_t od[] = {
#define OBJ(idx,sidx,d,i,t,rp,tp,r,w) { .id = #i, .index = idx, .subindex = sidx, .type = CiA309type(t), .size = sizeof(t), .parse = PARSE_FN(w,t), .print = PRINT_FN(r,t) },
#include "object_dictionary.h"
#undef OBJ
};

struct {
    const char* datatype;
    size_t size;
    void (*parse)(const char* str, void* buf);
    void (*print)(void* buf);
} by_type[] = {
    { "b",   1, parse_uint8_t,   print_uint8_t },
    { "i8",  1, parse_int8_t,    print_int8_t },
    { "u8",  1, parse_uint8_t,   print_uint8_t },
    { "x8",  1, parse_uint8_t,   print_x8_t },
    { "i16", 2, parse_int16_t,   print_int16_t },
    { "u16", 2, parse_uint16_t,  print_uint16_t },
    { "x16", 2, parse_uint16_t,  print_x16_t },
    { "i32", 4, parse_int32_t,   print_int32_t },
    { "u32", 4, parse_uint32_t,  print_uint32_t },
    { "x32", 4, parse_uint32_t,  print_x32_t },
    { "i64", 4, parse_int64_t,   print_int64_t },
    { "u64", 4, parse_uint64_t,  print_uint64_t },
    { "x64", 4, parse_uint64_t,  print_x64_t },
    { "vs",  4, parse_string4_t, print_string4_t },
};


// command context -------------------------------------------

// default_ctx is managed with set command
// ctx copies default values from default_ctx but current statement may override them 
console_context_t ctx;
static console_context_t default_ctx = {
    .sequence = 0,
    .net = 0,
    .node = 0,
    .sdo_timeout = &maxDelay,
    .sdo_block = false,
    .dump_msg = &enable_dump_msg,
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
    obj->print(value);
    printf("\r\n");
}

void print_result_ok(void) {
    printf("[%d] OK\r\n", ctx.sequence);
}

void print_result_error(const char *msg) {
    printf("[%d] ERROR:%s\r\n", ctx.sequence, msg);
}

object_dictionary_entry_t* get_dictionary_entry(const char* sym, const char* datatype)
{
    static object_dictionary_entry_t ret; // just one entry being read, no need to allocate
    ret.id = NULL;
    for (size_t i = 0; i < N_ELEMS(od); ++i) {
        if (0 == strcmp(sym, od[i].id)) {
            ret = od[i];
            break;
        }
    }

    if (ret.id == NULL) return NULL;
    if (datatype == NULL) return &ret;

    // type may be overriden by user (e.g. hex print)
    for (uint8_t i = 0; i < N_ELEMS(by_type); ++i) {
        if (0 == strcmp(datatype, by_type[i].datatype)) {
            ret.size = by_type[i].size;
            ret.parse = by_type[i].parse;
            ret.print = by_type[i].print;
            break;
        }
    }
    return &ret;
}

static object_dictionary_entry_t* get_object_entry(uint16_t index, uint8_t subindex, const char* datatype)
{
    static object_dictionary_entry_t ret; // just one entry being read, no need to allocate
    ret.index = index;
    ret.subindex = subindex;

    ret.parse = &parse_int32_t;
    ret.print = &print_x32_t;
    ret.size = 4;
    for (uint8_t i = 0; i < N_ELEMS(by_type); ++i) {
        if (0 == strcmp(datatype, by_type[i].datatype)) {
            ret.size = by_type[i].size;
            ret.parse = by_type[i].parse;
            ret.print = by_type[i].print;
            break;
        } 
    }
    return &ret;
}


static void read_object(object_dictionary_entry_t* obj) 
{
    object_value_t value;
    esp_err_t err = sdo_upload(0x600 + ctx.node, obj->index, obj->subindex, &value);
    if (err == ESP_OK) print_result_value(obj, &value);
    else print_result_error("Unsuccessful SDO upload");
}

static void write_object(object_dictionary_entry_t* obj, object_value_t* value) 
{ 
    esp_err_t err = sdo_download(0x600 + ctx.node, obj->index, obj->subindex, value, obj->size);
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
    const char *datatype = read_sym_args.datatype->count > 0 ? read_args.datatype->sval[0] : NULL;
    object_dictionary_entry_t* obj = get_dictionary_entry(sym, datatype);
    if (obj == NULL) print_result_error("Unknown object");
    else read_object(obj);
    return 0;
}

static int cmd_read(int argc, char **argv)
{
    char* arg_index = argv[1];
    if (argc > 1 && !isdigit((int)arg_index[0])) return cmd_read_sym(argc, argv);

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
    obj->parse(svalue, &value);
    write_object(obj, &value);
    return 0;
}

static int cmd_write(int argc, char **argv)
{
    char* arg_index = argv[1];
    if (argc > 1 && !isdigit((int)arg_index[0])) return cmd_write_sym(argc, argv);

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
    obj->parse(sval, &value);
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
        *default_ctx.sdo_timeout = pdMS_TO_TICKS(val);
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
            *default_ctx.dump_msg = true;
        }
        else {
            *default_ctx.dump_msg = false;
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
        print_result_int(pdTICKS_TO_MS(*ctx.sdo_timeout));
    }
    else if (strcasecmp(param, "sdo_block") == 0) {
        print_result_int(ctx.sdo_block);
    }
    else {
        print_result_error("unknown_param");
        return 1;
    }

    return 0;
}

static int cmd_exit(int argc, char **argv)
{
    epos_done();
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
                "  45071 Toledo, Spain\r\n"
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
                N "set network|node|sdo_timeout|sdo_block <value>\r\n"
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
            for (size_t i = 0; i < N_ELEMS(od); ++i) {
                object_dictionary_entry_t* o = &od[i];
                printf("  %04x %02x %-3s %c%c %s\r\n", o->index, o->subindex, o->type, R(o), W(o), o->id);
            }
        } else {
            printf( "Objects starting with %s:\r\n", topic);
            for (size_t i = 0; i < N_ELEMS(od); ++i) {
                object_dictionary_entry_t* o = &od[i];
                if (0 == strncmp(o->id, topic, strlen(topic)))
                    printf("  %04x %02x %-3s %c%c %s\r\n", o->index, o->subindex, o->type, (o->print?'R':' '), (o->parse?'W':' '), o->id);
            }
            return 1;
        }
    }

    return 0;
}

void epos_console_register_commands() 
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

    set_args.param = arg_str1(NULL, NULL, "<network|node|sdo_timeout|sdo_block>", "Parameter to set");
    set_args.value = arg_str1(NULL, NULL, "<value>", "Parameter value");
    set_args.end = arg_end(1);

    get_args.param = arg_str1(NULL, NULL, "<network|node|sdo_timeout|sdo_block>", "Parameter to get");
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

static void epos_console_run(char* line)
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
    if (isdigit((int)line[0])) {
        int net = strtol(line, NULL, 0);
        // Intentar siguiente token
        char* next = strtok(line, " \t");
        if (next && isdigit((int)next[0])) {
            ctx.net = net;
            ctx.node = strtol(next, NULL, 0);
            line = strtok(NULL, ""); // el resto de la línea
        } else if (0 == strncmp("set", next, 3)) { 
            // en set el número es [net]
            ctx.net = net;
        } else {
            ctx.node = net;
        }
    }

    int ret;
    esp_console_run(line, &ret);
}

static void suggest_completions(const char *cmd, const char* sym, linenoiseCompletions *lc, bool read, bool write) 
{
    size_t len = strlen(sym);
    if (len == 0) return;
    for (size_t i = 0; i < N_ELEMS(od); i++) {
        bool maybe = (read && od[i].print) || (write && od[i].parse);
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

static void epos_initialize_console(const epos_console_cfg_t* cfg)
{
    // En USB-SJTAG el cfg sólo lo usaremos para tamaños de buffer y tal.
    if (cfg == NULL) return;

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

    int probe_status = linenoiseProbe();
    if (probe_status) {
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing disabled.\n");
        linenoiseSetDumbMode(1);
    }

}



void epos_console_task(void *arg)
{
    const epos_console_cfg_t* cfg = (const epos_console_cfg_t*)arg;
    if (cfg == NULL) {
        printf("\n"
               "No hay config, usando default.\n");
        static epos_console_cfg_t fallback_cfg = EPOS_CONSOLE_DEFAULT();
        cfg = &fallback_cfg;
    }
    epos_initialize_console(cfg);
    for(;;) {
        char* line = linenoise("> ");
        if (line == NULL) continue;
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            epos_console_run(line);
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

    // Crear el socket de escucha
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Error socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Configurar la dirección del servidor
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);

    // Asociar el socket a la dirección y puerto
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Error bind: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // Escuchar conexiones entrantes (se admite 1 conexión en cola)
    if (listen(listen_sock, 1) < 0) {
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

    // Enviar un prompt inicial al cliente
    send(client_sock, "> ", strlen("> "), 0);

    // Bucle principal: leer datos del cliente y procesarlos
    while (1) {
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
        }

        // Procesar el comando si no está vacío
        if (strlen(rx_buffer) > 0) {
            // Aquí podrías agregar el comando a un historial si fuera necesario
            epos_console_run(rx_buffer);
        }
        // Enviar nuevamente el prompt al cliente
        send(client_sock, "> ", strlen("> "), 0);
    }

    // Cerrar conexiones y liberar recursos
    close(client_sock);
    close(listen_sock);
    vTaskDelete(NULL);
}
