#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
    This header no longer includes a fixed object dictionary.

    The application provides its own .h/.c files and expands these macros over
    its own OBJ(...) table.

    OBJ(index, subindex, "Description", symbol, c_type, rx_pdo, tx_pdo, getter, setter)
*/

#define CANOPEN_CLIENT_DECLARE_GETTER(fn_name, idx, subidx, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (esp_err_t fn_name(uint8_t n, T *value);)

#define CANOPEN_CLIENT_DECLARE_SETTER(fn_name, idx, subidx, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (esp_err_t fn_name(uint8_t n, T value);)

#define CANOPEN_CLIENT_DEFINE_GETTER(fn_name, idx, subidx, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (esp_err_t fn_name(uint8_t n, T *value) { return sdo_upload(0x600u + n, idx, subidx, value); })

#define CANOPEN_CLIENT_DEFINE_SETTER(fn_name, idx, subidx, T) \
    IF_ELSE(IS_NA(fn_name)) \
    ( ) \
    (esp_err_t fn_name(uint8_t n, T value) { return sdo_download(0x600u + n, idx, subidx, &value, sizeof(T)); })

#define CANOPEN_PDO_MAP_ENTRY(idx, subidx, T) (((uint32_t)(idx) << 16) | ((uint32_t)(subidx) << 8) | (uint32_t)(sizeof(T) * 8u))

#define CANOPEN_CLIENT_DECLARE_TX_MAPPING_ENUM(symbol, enable, idx, subidx, T) \
    IF_ELSE(enable) \
    ( txpdo_##symbol = CANOPEN_PDO_MAP_ENTRY(idx, subidx, T), ) \
    ( )

#define CANOPEN_CLIENT_DECLARE_RX_MAPPING_ENUM(symbol, enable, idx, subidx, T) \
    IF_ELSE(enable) \
    ( rxpdo_##symbol = CANOPEN_PDO_MAP_ENTRY(idx, subidx, T), ) \
    ( )


// Convenience macros to define the client API from an OBJ(...) table, reusing the same getter/setter names and PDO mapping flags.
#define OBJ_CLIENT_DECLARE_GETTER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DECLARE_GETTER(getter, index, subindex, type)
#define OBJ_CLIENT_DECLARE_SETTER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DECLARE_SETTER(setter, index, subindex, type)
#define OBJ_CLIENT_DECLARE_TX_MAPPING_ENUM(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DECLARE_TX_MAPPING_ENUM(symbol, txpdo, index, subindex, type)
#define OBJ_CLIENT_DECLARE_RX_MAPPING_ENUM(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DECLARE_RX_MAPPING_ENUM(symbol, rxpdo, index, subindex, type)
#define OBJ_CLIENT_DEFINE_GETTER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DEFINE_GETTER(getter, index, subindex, type)
#define OBJ_CLIENT_DEFINE_SETTER(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DEFINE_SETTER(setter, index, subindex, type)

#define OBJ_CLIENT_DECLARE(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DECLARE_GETTER(getter, index, subindex, type) \
    CANOPEN_CLIENT_DECLARE_SETTER(setter, index, subindex, type)
#define OBJ_CLIENT_DEFINE(index, subindex, description, symbol, type, rxpdo, txpdo, getter, setter) \
    CANOPEN_CLIENT_DEFINE_GETTER(getter, index, subindex, type) \
    CANOPEN_CLIENT_DEFINE_SETTER(setter, index, subindex, type)

#ifdef __cplusplus
}
#endif
