#pragma once

# include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int sequence;
    int net;
    int node;
    TickType_t* sdo_timeout;
    bool sdo_block;     // falso por defecto
    bool* dump_msg;
} console_context_t;

typedef struct {
    const char* id;
    uint16_t index;
    uint8_t subindex;
    const char* type;
    size_t size;
    void (*parse)(const char* str, void* buf);
    void (*print)(void* buf);
} object_dictionary_entry_t;

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

#ifdef __cplusplus
}
#endif
