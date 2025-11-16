#ifndef EPOS_H
#define EPOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "canopen_types.h"
#include "esp_err.h"
#include "driver/twai.h"

#define DEFAULT_CAN_TX 22
#define DEFAULT_CAN_RX 21

esp_err_t nmt_enter_preoperational(uint8_t node);
esp_err_t nmt_reset_communication(uint8_t node);
esp_err_t nmt_reset_node(uint8_t node);
esp_err_t nmt_start_remote_node(uint8_t node);
esp_err_t nmt_stop_remote_node(uint8_t node);

esp_err_t sdo_download(uint32_t id, uint16_t index, uint8_t subindex, void* value, size_t size);
esp_err_t sdo_upload(uint32_t id, uint16_t index, uint8_t subindex, void* ret);
esp_err_t nmt(uint8_t cs, uint8_t n);
void epos_done(void);
void epos_set_tx_rx_pins(gpio_num_t tx, gpio_num_t rx);
esp_err_t epos_initialize(bool activate_console);
esp_err_t epos_wait_done(void);
esp_err_t epos_wait_until(uint32_t cobid, void* ret);

typedef void (*canopen_handler_fn)(uint32_t cobid, void* msg, void* context);
esp_err_t epos_register_canopen_handler(uint32_t cobid, canopen_handler_fn handler_fn, void* context);
esp_err_t epos_unregister_canopen_handler(uint32_t cobid);

typedef union { char str[4]; uint32_t data; } string4_t; 
typedef union { char str[8]; uint64_t data; } string8_t; 

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
#include "object_dictionary.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SETTER_DECL(w,id,sid,t)
#include "object_dictionary.h"
#undef OBJ

typedef enum {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_MAPPING_OBJECT(tx,tp,i,id,sid,t)
#include "object_dictionary.h"
#undef OBJ
} pdo_tx_mapping_object_t;

typedef enum {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_MAPPING_OBJECT(rx,rp,i,id,sid,t)
#include "object_dictionary.h"
#undef OBJ
} pdo_rx_mapping_object_t;


// private
extern TickType_t maxDelay;
extern bool enable_dump_msg;

#define EPOS_ERROR_CHECK(x) do {                                         \
        esp_err_t err_rc_ = (x);                                        \
        if (unlikely(err_rc_ != ESP_OK)) {                              \
            _esp_error_check_failed_without_abort(err_rc_, __FILE__, __LINE__, __func__, #x); \
        }                                                               \
    } while(0)

#ifdef __cplusplus
}
#endif

#endif

