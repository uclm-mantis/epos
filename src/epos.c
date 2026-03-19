#include "epos.h"
#include "canopen.h"
#include "canopen_console.h"

static canopen_console_cfg_t console_cfg = CANOPEN_CONSOLE_DEFAULT();

esp_err_t epos_initialize(const epos_init_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = canopen_initialize(&cfg->canopen);
    if (err != ESP_OK) {
        return err;
    }

    console_cfg = cfg->console;

    if (cfg->enable_console) {
        xTaskCreatePinnedToCore(canopen_console_task,
                                 "canopen_console",
                                 4096,
                                 (void *)&console_cfg,
                                 10,
                                 NULL,
                                 tskNO_AFFINITY);
    }

    return ESP_OK;
}

esp_err_t epos_wait_done(void)
{
    return canopen_wait_done();
}

esp_err_t epos_wait_until(uint32_t cobid, void* ret)
{
    return canopen_wait_until(cobid, ret);
}

void epos_done(void)
{
    canopen_done();
}

esp_err_t epos_register_canopen_handler(uint32_t cobid,
    canopen_handler_fn handler_fn,
    void *context,
    epos_canopen_handler_handle_t *out_handle)
{
    return canopen_register_handler(cobid, handler_fn, context, out_handle);
}

esp_err_t epos_unregister_canopen_handler(epos_canopen_handler_handle_t handle)
{
    return canopen_unregister_handler(handle);
}

