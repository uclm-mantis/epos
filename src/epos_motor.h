#pragma once

#include "canopen_client.h"
#include "maxon_EPOS2_od.h"

#ifdef __cplusplus
extern "C" {
#endif

MAXON_EPOS2_OD(OBJ_CLIENT_DECLARE)

typedef enum {
    MAXON_EPOS2_OD(OBJ_CLIENT_DECLARE_TX_MAPPING_ENUM)
} motor_tpdo_map_t;

typedef enum {
    MAXON_EPOS2_OD(OBJ_CLIENT_DECLARE_RX_MAPPING_ENUM)
} motor_rpdo_map_t;

#ifdef __cplusplus
}
#endif
