#pragma once

#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline esp_err_t epos_initialize(const canopen_init_cfg_t *cfg)
{
    return canopen_initialize(cfg);
}

static inline void epos_done(void)
{
    canopen_request_shutdown();
}

static inline esp_err_t epos_wait_shutdown(void)
{
    return canopen_wait_shutdown();
}

static inline esp_err_t epos_wait_until(uint32_t cobid, void *ret)
{
    return canopen_wait_until(cobid, ret);
}

#ifdef __cplusplus
}
#endif
