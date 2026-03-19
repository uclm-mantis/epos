#pragma once

#include "canopen.h"
#include "canopen_console.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    canopen_init_cfg_t canopen;
    bool enable_console;
    canopen_console_cfg_t console;
} epos_init_cfg_t;

#define EPOS_INIT_DEFAULT() { \
    .canopen = CANOPEN_INIT_DEFAULT(), \
    .enable_console = true, \
    .console = { .rx_buffer_size = 256, .tx_buffer_size = 256 }, \
}

static inline epos_init_cfg_t epos_init_default(void)
{
    const epos_init_cfg_t cfg = EPOS_INIT_DEFAULT();
    return cfg;
}

typedef canopen_handler_handle_t epos_canopen_handler_handle_t;

esp_err_t epos_initialize(const epos_init_cfg_t *cfg);
esp_err_t epos_wait_done(void);
esp_err_t epos_wait_until(uint32_t cobid, void* ret);
void epos_done(void);

esp_err_t epos_register_canopen_handler(uint32_t cobid,
                                        canopen_handler_fn handler_fn,
                                        void *context,
                                        epos_canopen_handler_handle_t *out_handle);
esp_err_t epos_unregister_canopen_handler(epos_canopen_handler_handle_t handle);

#ifdef __cplusplus
}
#endif
