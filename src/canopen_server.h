#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CANOPEN_OD_OK = 0,
    CANOPEN_OD_UNSUPPORTED_ACCESS,
    CANOPEN_OD_WRITEONLY,
    CANOPEN_OD_READONLY,
    CANOPEN_OD_NO_SUCH_OBJECT,
    CANOPEN_OD_NO_SUCH_SUBINDEX,
    CANOPEN_OD_TYPE_MISMATCH,
    CANOPEN_OD_INVALID_VALUE,
    CANOPEN_OD_DATA_TOO_LONG,
    CANOPEN_OD_DATA_TOO_SHORT,
    CANOPEN_OD_HW_ERROR,
} canopen_od_status_t;

uint32_t canopen_od_status_to_abort_code(canopen_od_status_t st);

typedef canopen_od_status_t (*canopen_server_get_fn_t)(void *value);
typedef canopen_od_status_t (*canopen_server_set_fn_t)(const void *value);

typedef struct {
    uint16_t index;
    uint8_t subindex;
    size_t size;
    canopen_server_get_fn_t get;
    canopen_server_set_fn_t set;
} canopen_server_od_entry_t;

/* Typed callback declarations for application tables. */
#define CANOPEN_SERVER_DECLARE_GETTER(fn_name, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (canopen_od_status_t on_##fn_name(T *value);)

#define CANOPEN_SERVER_DECLARE_SETTER(fn_name, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (canopen_od_status_t on_##fn_name(T value);)

/* Wrappers used by the generated OD table. */
#define CANOPEN_SERVER_DEFINE_GETTER_WRAPPER(fn_name, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (static inline canopen_od_status_t wrap_##fn_name(void *value) { return on_##fn_name((T *)value); })

#define CANOPEN_SERVER_DEFINE_SETTER_WRAPPER(fn_name, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (static inline canopen_od_status_t wrap_##fn_name(const void *value) { return on_##fn_name(*(const T *)value); })

#define CANOPEN_SERVER_GETTER_WRAPPER(fn_name) \
    IF_ELSE(IS_NA(fn_name)) \
    ( NULL ) \
    ( &wrap_##fn_name )

#define CANOPEN_SERVER_SETTER_WRAPPER(fn_name) \
    IF_ELSE(IS_NA(fn_name)) \
    ( NULL ) \
    ( &wrap_##fn_name )

#define CANOPEN_SERVER_OD_ENTRY(idx, subidx, T, getter, setter) \
    { \
        .index = (idx), \
        .subindex = (subidx), \
        .size = sizeof(T), \
        .get = CANOPEN_SERVER_GETTER_WRAPPER(getter), \
        .set = CANOPEN_SERVER_SETTER_WRAPPER(setter), \
    }

// convenience macros to define the OD from an OBJ(...) table, reusing the same getter/setter names
#define OBJ_SERVER_DECLARE_GETTER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_DECLARE_GETTER(getter, type)
#define OBJ_SERVER_DECLARE_SETTER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_DECLARE_SETTER(setter, type)

#define OBJ_SERVER_DEFINE_GETTER_WRAPPER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_DEFINE_GETTER_WRAPPER(getter, type)
#define OBJ_SERVER_DEFINE_SETTER_WRAPPER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_DEFINE_SETTER_WRAPPER(setter, type)

#define OBJ_SERVER_GETTER_WRAPPER(idx, subidx, desc, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_GETTER_WRAPPER(getter, type)
#define OBJ_SERVER_SETTER_WRAPPER(idx, subidx, desc, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_SETTER_WRAPPER(setter, type)

#define OBJ_SERVER_OD_ENTRY(idx, subidx, desc, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_SERVER_OD_ENTRY(idx, subidx, type, getter, setter),

/* Explicit server instance: the application passes the OD at startup. */
typedef struct {
    bool in_use;
    uint8_t node_id;
    uint32_t cobid_req;
    uint32_t cobid_resp;
    canopen_handler_handle_t handle;

    const canopen_server_od_entry_t *od;
    size_t od_len;

    uint8_t state;
    uint16_t index;
    uint8_t subindex;
    uint8_t toggle;
    uint8_t buf[256];
    size_t size;
    size_t offset;
} canopen_server_t;

esp_err_t canopen_server_start(canopen_server_t *server,
                               uint8_t node_id,
                               const canopen_server_od_entry_t *od,
                               size_t od_len);

esp_err_t canopen_server_stop(canopen_server_t *server);

esp_err_t canopen_server_dispatch_sdo(canopen_server_t *server,
                                      uint32_t cobid_req,
                                      const void *req,
                                      size_t req_size,
                                      uint32_t *cobid_resp,
                                      void *resp,
                                      size_t *resp_size);

canopen_od_status_t canopen_server_od_get(const canopen_server_od_entry_t *od,
                                          size_t od_len,
                                          uint16_t index,
                                          uint8_t subindex,
                                          void *value,
                                          size_t size);

canopen_od_status_t canopen_server_od_set(const canopen_server_od_entry_t *od,
                                          size_t od_len,
                                          uint16_t index,
                                          uint8_t subindex,
                                          const void *value,
                                          size_t size);

const canopen_server_od_entry_t *canopen_server_find(const canopen_server_od_entry_t *od,
                                                     size_t od_len,
                                                     uint16_t index,
                                                     uint8_t subindex);

#ifdef __cplusplus
}
#endif
