#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IS_NA(x) IS_NA_HELPER(IS_NA_PRIMITIVE_CAT(IS_NA_CHECK_, x))
#define IS_NA_PRIMITIVE_CAT(a, b) a ## b
#define IS_NA_CHECK_NA PROBE()
#define PROBE() ~, 1
#define IS_NA_HELPER(...) IS_NA_HELPER_(__VA_ARGS__, 0)
#define IS_NA_HELPER_(a, b, ...) b

#define IF_ELSE(condition) CAT(IF_, condition)
#define IF_1(...) __VA_ARGS__ IF_1_ELSE
#define IF_0(...)             IF_0_ELSE
#define IF_1_ELSE(...)
#define IF_0_ELSE(...) __VA_ARGS__
#define CAT(a, b) a ## b

#define GENERATE_GETTER_DECL(x, idx, subidx, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (esp_err_t x(uint8_t n, T* value);)
#define GENERATE_SETTER_DECL(x, idx, subidx, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (esp_err_t x(uint8_t n, T value);)

#define GENERATE_GETTER(x, idx, subidx, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (esp_err_t x(uint8_t n, T* value) { return sdo_upload(0x600 + n, idx, subidx, value); })
#define GENERATE_SETTER(x, idx, subidx, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (esp_err_t x(uint8_t n, T value) { return sdo_download(0x600 + n, idx, subidx, &value, sizeof(T)); })

#define PARSE_FN(x,T) \
    IF_ELSE(IS_NA(x)) \
    ( NULL ) \
    ( &parse_##T )
#define PRINT_FN(x,T) \
    IF_ELSE(IS_NA(x)) \
    ( NULL ) \
    ( &print_##T )

#define GENERATE_MAPPING_OBJECT(prefix, enable, id, idx, subidx, T) \
    IF_ELSE(enable) \
    ( prefix##pdo_##id = ((idx<<16) | (subidx<<8) | (sizeof(T)<<3)), ) \
    ( )

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_GETTER_DECL(r,id,sid,t)
#include "client_od.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SETTER_DECL(w,id,sid,t)
#include "client_od.h"
#undef OBJ

typedef enum {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_MAPPING_OBJECT(tx,tp,i,id,sid,t)
#include "client_od.h"
#undef OBJ
} pdo_tx_mapping_object_t;

typedef enum {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_MAPPING_OBJECT(rx,rp,i,id,sid,t)
#include "client_od.h"
#undef OBJ
} pdo_rx_mapping_object_t;

#ifdef __cplusplus
}
#endif
