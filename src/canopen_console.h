#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char* datatype;
    size_t size;
    void (*parse)(const char* str, void* buf);
    void (*print)(void* buf);
} canopen_type_entry_t;

typedef struct {
    const char* id;
    uint16_t index;
    uint8_t subindex;
    canopen_type_entry_t* type;
    bool readable;
    bool writable;
} object_dictionary_entry_t;

typedef struct {
    int rx_buffer_size;
    int tx_buffer_size;
    bool dumb_mode;
    bool enable_usb_console;
    bool enable_tcp_console;
    object_dictionary_entry_t* object_dictionary;
    size_t object_dictionary_entries;
} canopen_console_cfg_t;

#define CANOPEN_CONSOLE_DEFAULT() { \
    .rx_buffer_size = 256, \
    .tx_buffer_size = 256, \
    .dumb_mode = false, \
    .enable_usb_console = true, \
    .enable_tcp_console = false, \
    .object_dictionary = NULL, \
    .object_dictionary_entries = 0, \
}

typedef struct {
    int sequence;
    int net;
    int node;
    TickType_t sdo_timeout;
    bool sdo_block;     // falso por defecto
    bool dump_msg;
} console_context_t;

typedef union {
    bool b;
    uint8_t u8;
    uint16_t u16;
    uint32_t u32;
    uint64_t u64;
    int8_t i8;
    int16_t i16;
    int32_t i32;
    int64_t i64;
} object_value_t;

extern console_context_t ctx;

void print_result_int(int value);
void print_result_value(object_dictionary_entry_t* obj, object_value_t* value);
void print_result_ok(void);
void print_result_error(const char *msg);
object_dictionary_entry_t* get_dictionary_entry(const char* sym, const char* datatype);
void canopen_console_register_commands(void);
void canopen_console_init(const canopen_console_cfg_t* cfg);

// T = Console Type: defines a type that can be used as a parameter in console commands, with printf/scanf format specifiers.
// A = Console Type Alias: defines an alias for a console type, with a custom printf/scanf format specifier.
// S = Console Type String: defines a string type.
#define CANOPEN_CONSOLE_TYPES(T, A, S) \
    T(b,   bool,     ul,  d) \
    T(i8,  int8_t,   l,   d) \
    T(u8,  uint8_t,  ul,  u) \
    A(x8,  uint8_t,  ul,  02x) \
    T(i16, int16_t,  l,   d) \
    T(u16, uint16_t, ul,  u) \
    A(x16, uint16_t, ul,  04x) \
    T(i32, int32_t,  l,   ld) \
    T(u32, uint32_t, ul,  lu) \
    A(x32, uint32_t, ul,  08lx) \
    T(i64, int64_t,  ll,  lld) \
    T(u64, uint64_t, ull, llu) \
    A(x64, uint64_t, ull, 016llx) \
    S(vs,  string8_t)

// type descriptor table
extern canopen_type_entry_t by_type[];

 // enumeration for type descriptors
#define ENTRY_T_DEF(datatype, ctype, scan_sfx, print_sfx) datatype##_info,
#define ENTRY_A_DEF(datatype, ctype, scan_sfx, print_sfx) datatype##_info,
#define ENTRY_S_DEF(datatype, ctype) datatype##_type_info,
enum {
    CANOPEN_CONSOLE_TYPES(ENTRY_T_DEF, ENTRY_A_DEF, ENTRY_S_DEF)
};

#define OBJ_CONSOLE_OD_ENTRY(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    { \
        .id = #symbol, \
        .index = index, \
        .subindex = subindex, \
        .type = &by_type[type##_type_info], \
        .readable = (getter != NULL), \
        .writable = (setter != NULL), \
    },

// instantiates an object dictionary array from a CANOPEN_OD table, to be passed to the console at init
#define CANOPEN_CONSOLE_OBJECT_DICTIONARY(CANOPEN_OD, name) \
    const object_dictionary_entry_t name[] = { \
        CANOPEN_OD(OBJ_CONSOLE_OD_ENTRY) \
    }; \
    const size_t name##_len = sizeof(name) / sizeof(name[0]);

#ifdef __cplusplus
}
#endif
