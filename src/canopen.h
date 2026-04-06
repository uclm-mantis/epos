#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_CAN_TX 21
#define DEFAULT_CAN_RX 20
#define DEFAULT_CAN_BITRATE 1000000u

static inline twai_message_t twai_message_init_std(uint32_t identifier, const void *data, size_t len)
{
    twai_message_t msg = {
        .identifier = identifier,
        .data_length_code = (uint8_t)len,
        .extd = false,
        .rtr = false,
        .self = false,
        .ss = false,
        .dlc_non_comp = false,
        .data = {0},
    };
    if (data != NULL && len > 0 && len <= sizeof(msg.data)) {
        memcpy(msg.data, data, len);
    }
    return msg;
}

typedef void (*canopen_handler_fn)(uint32_t cobid, void *data, void *context);

typedef struct {
    int can_tx_pin;
    int can_rx_pin;
    uint32_t can_bitrate;
    unsigned max_delay_ms;
    bool enable_dump_msg;
} canopen_init_cfg_t;

#define CANOPEN_INIT_DEFAULT() (canopen_init_cfg_t){ \
    .can_tx_pin = DEFAULT_CAN_TX, \
    .can_rx_pin = DEFAULT_CAN_RX, \
    .can_bitrate = DEFAULT_CAN_BITRATE, \
    .max_delay_ms = 3000, \
    .enable_dump_msg = false, \
}

static inline canopen_init_cfg_t canopen_init_default(void) { return CANOPEN_INIT_DEFAULT(); }

/* -------------------------------------------------------------------------- */
/*  Core lifecycle                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t canopen_initialize(const canopen_init_cfg_t *cfg);
void canopen_request_shutdown(void);
esp_err_t canopen_wait_shutdown(void);

/* Wait synchronously for one frame with the given COB-ID. */
esp_err_t canopen_wait_until(uint32_t cobid, void *ret);

/* -------------------------------------------------------------------------- */
/*  Asynchronous subscriptions                                                */
/* -------------------------------------------------------------------------- */

typedef struct canopen_handler_entry *canopen_handler_handle_t;

esp_err_t canopen_register_handler(uint32_t cobid,
                                   canopen_handler_fn handler_fn,
                                   void *context,
                                   canopen_handler_handle_t *out_handle);
esp_err_t canopen_unregister_handler(canopen_handler_handle_t handle);

/* -------------------------------------------------------------------------- */
/*  Raw CAN helpers                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t canopen_send(const twai_message_t *msg);   /* synchronous: waits until TX task handles it */
esp_err_t canopen_post(const twai_message_t *msg);   /* asynchronous: enqueue and return */

/* -------------------------------------------------------------------------- */
/*  NMT                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t nmt(uint8_t cs, uint8_t n);
static inline esp_err_t nmt_enter_preoperational(uint8_t node) { return nmt(NMT_ENTER_PREOPERATIONAL, node); }
static inline esp_err_t nmt_reset_communication(uint8_t node)  { return nmt(NMT_RESET_COMMUNICATION, node); }
static inline esp_err_t nmt_reset_node(uint8_t node)           { return nmt(NMT_RESET_NODE, node); }
static inline esp_err_t nmt_start_remote_node(uint8_t node)    { return nmt(NMT_START_REMOTE_NODE, node); }
static inline esp_err_t nmt_stop_remote_node(uint8_t node)     { return nmt(NMT_STOP_REMOTE_NODE, node); }

/* -------------------------------------------------------------------------- */
/*  SDO client                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t sdo_download(uint32_t id, uint16_t index, uint8_t subindex, void *value, size_t size);
esp_err_t sdo_upload(uint32_t id, uint16_t index, uint8_t subindex, void *ret);

/* -------------------------------------------------------------------------- */
/*  PDO client convenience                                                    */
/* -------------------------------------------------------------------------- */

typedef enum {
    CANOPEN_PDO_RX = 0,
    CANOPEN_PDO_TX = 1,
} canopen_pdo_dir_t;

typedef struct {
    uint8_t data[8];
    uint8_t len;
} canopen_pdo_payload_t;

/* Mapping entry format: 0xIIII SS LL = (index << 16) | (subindex << 8) | bit_length */
typedef struct {
    uint32_t cob_id;              /* 0 = use default COB-ID */
    uint8_t transmission_type;    /* 1..240 synchronous, 254/255 asynchronous */
    uint16_t inhibit_time;        /* only used for TPDO */
    const uint32_t *mapped;       /* array of mapping entries */
    uint8_t mapped_count;         /* number of mapping entries */
} canopen_pdo_cfg_t;

esp_err_t canopen_pdo_configure(uint8_t node,
                                canopen_pdo_dir_t dir,
                                uint8_t pdo_num,
                                const canopen_pdo_cfg_t *cfg);

esp_err_t canopen_pdo_send(uint8_t node,
                           uint8_t rpdo_num,
                           uint32_t cob_id_override,
                           const void *data,
                           size_t len);

esp_err_t canopen_pdo_post(uint8_t node,
                           uint8_t rpdo_num,
                           uint32_t cob_id_override,
                           const void *data,
                           size_t len);

esp_err_t canopen_pdo_subscribe(uint8_t node,
                                uint8_t tpdo_num,
                                uint32_t cob_id_override,
                                canopen_handler_fn fn,
                                void *context,
                                canopen_handler_handle_t *out);

void canopen_pdo_payload_clear(canopen_pdo_payload_t *p);
esp_err_t canopen_pdo_payload_put_u8(canopen_pdo_payload_t *p, uint8_t v);
esp_err_t canopen_pdo_payload_put_u16(canopen_pdo_payload_t *p, uint16_t v);
esp_err_t canopen_pdo_payload_put_u32(canopen_pdo_payload_t *p, uint32_t v);
esp_err_t canopen_pdo_payload_put_i32(canopen_pdo_payload_t *p, int32_t v);

/* -------------------------------------------------------------------------- */
/*  Runtime parameters                                                        */
/* -------------------------------------------------------------------------- */

TickType_t canopen_get_max_delay_ms(void);
void canopen_max_delay_ms(unsigned delay);

bool canopen_is_dump_enabled(void);
void canopen_dump_enabled(bool enable);

/* -------------------------------------------------------------------------- */
/*  Small preprocessor helpers reused by generated APIs                       */
/* -------------------------------------------------------------------------- */

#define CANOPEN_ERROR_CHECK(x) do { \
        esp_err_t err_rc_ = (x); \
        if (unlikely(err_rc_ != ESP_OK)) { \
            _esp_error_check_failed_without_abort(err_rc_, __FILE__, __LINE__, __func__, #x); \
        } \
    } while(0)

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

#ifdef __cplusplus
}
#endif
