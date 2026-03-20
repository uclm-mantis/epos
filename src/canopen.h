#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "canopen_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_CAN_TX 17
#define DEFAULT_CAN_RX 18

typedef struct {
    int can_tx_pin;
    int can_rx_pin;
    unsigned max_delay_ms;
    bool enable_dump_msg;
} canopen_init_cfg_t;

#define CANOPEN_INIT_DEFAULT() (canopen_init_cfg_t){ \
    .can_tx_pin = DEFAULT_CAN_TX, \
    .can_rx_pin = DEFAULT_CAN_RX, \
    .max_delay_ms = 3000, \
    .enable_dump_msg = false, \
}

static inline canopen_init_cfg_t canopen_init_default(void)    { return CANOPEN_INIT_DEFAULT(); }
esp_err_t canopen_initialize(const canopen_init_cfg_t *cfg);
void canopen_request_shutdown(void);
esp_err_t canopen_wait_shutdown(void);
esp_err_t canopen_wait_until(uint32_t cobid, void* ret);

typedef struct canopen_handler_entry* canopen_handler_handle_t;
esp_err_t canopen_register_handler(uint32_t cobid,
                                   canopen_handler_fn handler_fn,
                                   void *context,
                                   canopen_handler_handle_t *out_handle);
esp_err_t canopen_unregister_handler(canopen_handler_handle_t handle);

esp_err_t nmt(uint8_t cs, uint8_t n);
static inline esp_err_t nmt_enter_preoperational(uint8_t node) { return nmt(NMT_ENTER_PREOPERATIONAL, node); }
static inline esp_err_t nmt_reset_communication(uint8_t node)  { return nmt(NMT_RESET_COMMUNICATION, node); }
static inline esp_err_t nmt_reset_node(uint8_t node)           { return nmt(NMT_RESET_NODE, node); }
static inline esp_err_t nmt_start_remote_node(uint8_t node)    { return nmt(NMT_START_REMOTE_NODE, node); }
static inline esp_err_t nmt_stop_remote_node(uint8_t node)     { return nmt(NMT_STOP_REMOTE_NODE, node); }

esp_err_t sdo_download(uint32_t id, uint16_t index, uint8_t subindex, void* value, size_t size);
esp_err_t sdo_upload(uint32_t id, uint16_t index, uint8_t subindex, void* ret);

TickType_t canopen_get_max_delay_ms(void);
void canopen_max_delay_ms(unsigned delay);

bool canopen_is_dump_enabled(void);
void canopen_dump_enabled(bool enable);

#define CANOPEN_ERROR_CHECK(x) do { \
        esp_err_t err_rc_ = (x); \
        if (unlikely(err_rc_ != ESP_OK)) { \
            _esp_error_check_failed_without_abort(err_rc_, __FILE__, __LINE__, __func__, #x); \
        } \
    } while(0)

#ifdef __cplusplus
}
#endif
