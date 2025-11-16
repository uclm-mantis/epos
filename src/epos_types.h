#ifndef EPOS_TYPES_H
#define EPOS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef union {
    uint8_t value;
    struct {
        uint8_t s : 7;
        uint8_t r : 1;
    };
} NMT_heartbeat_t;

typedef enum : uint8_t {
    NMT_HB_boot_up = 0,
    NMT_HB_stopped = 4,
    NMT_HB_operational = 5,
    NMT_HB_preoperational = 127,
} NMT_HB_state_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t switch_on : 1;
        uint16_t enable_voltage : 1;
        uint16_t quickstop : 1;
        uint16_t enable_operation : 1;
        uint16_t __reserved1 : 3;
        uint16_t fault_reset : 1;
        uint16_t halt : 1;
        uint16_t __reserved2 : 7;
    };
    struct {
        uint16_t all1 : 4;
        uint16_t new_setpoint : 1;
        uint16_t ch_set_immediate : 1;
        uint16_t relative : 1;
        uint16_t __ppm2 : 8;
        uint16_t endless_move : 1;

    } ppm;
    struct {
        uint16_t all1: 4;
        uint16_t homing_start : 1;
        uint16_t __hmm2: 11;
    } hmm;
    struct {
        uint16_t all1: 4;
        uint16_t enable_ipm : 1;
        uint16_t __ipm2: 11;
    } ipm;
} ControlWord_t;

typedef enum : int8_t {
    INTERPOLATED_POSITION_MODE = 7,
    HOMING_MODE = 6,
    PROFILE_VELOCITY_MODE = 3,
    PROFILE_POSITION_MODE = 1,
    UNKNOWN_MODE = 0,
    POSITION_MODE = -1,
    VELOCITY_MODE = -2,
    CURRENT_MODE = -3,
    DIAGNOSTIC_MODE = -4,
    MASTER_ENCODER_NODE = -5,
    STEP_DIRECTION_MODE = -6
} OperationMode_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t ready_switch_on : 1;
        uint16_t switched_on : 1;
        uint16_t operation_enable : 1;
        uint16_t fault : 1;
        uint16_t voltage_enabled : 1;
        uint16_t quickstop : 1;
        uint16_t switch_on_disable : 1;
        uint16_t warning : 1;
        uint16_t offset_current_measured : 1;
        uint16_t remote : 1;
        uint16_t target_reached : 1;
        uint16_t internal_limit_active : 1;
        uint16_t __reserved1 : 2;
        uint16_t refresh_cycle : 1;
        uint16_t referenced_to_home : 1;
    };
    struct {
        uint16_t __ppm1: 12;
        uint16_t setpoint_ack : 1;
        uint16_t following_error : 1;
        uint16_t __ppm2: 2;
    } ppm;
    struct{
        uint16_t __pvm1: 12;
        uint16_t speed : 1;
        uint16_t __pvm2: 3;

    } pvm;
    struct {
        uint16_t __hmm1: 12;
        uint16_t homing_attained : 1;
        uint16_t homing_error : 1;
        uint16_t __hmm2: 2;
    } hmm;
    struct{
        uint16_t __ipm1: 12;
        uint16_t ipm_active : 1;
        uint16_t __ipm2: 3;

    } ipm;
} StatusWord_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t generic_error : 1;
        uint8_t current_error : 1;
        uint8_t voltage_error : 1;
        uint8_t temperature_error : 1;
        uint8_t communication_error : 1;
        uint8_t profile_specific : 1;
        uint8_t reserved : 1;
        uint8_t motion_error : 1;
    };
} ErrorRegister_t;

typedef union {
    uint8_t value;
    struct {
        uint32_t heartbeat_time : 16;
        uint32_t node : 8;
        uint32_t reserved : 8;
    };
} ConsumerHeartBeatTime_t;

typedef union {
    uint8_t value;
    struct {
        uint32_t can_id : 11;
        uint32_t can_base_frame : 19; // 0
        uint32_t no_rtr : 1;
        uint32_t no_valid : 1;
    };
} PDO_COBid_t;

typedef enum : uint8_t {
    SYNCHRONOUS = 1,
    ASYNCHRONOUS_ON_RTR = 253,
    ASYNCHRONOUS = 255,
} PDO_Type_t;

typedef enum : uint16_t {
    PHASE_MODULATED_DC_MOTOR = 1,
    SINUSOIDAL_PM_BL_MOTOR = 10,
    TRAPEZOIDAL_PM_BL_MOTOR = 11,
    MANUFACTURER_SPECIFIC = 65535,
} MotorType_t;

typedef enum : uint16_t {
    CAN_BITRATE_1MBPS = 0,
    CAN_BITRATE_800KBPS = 1,
    CAN_BITRATE_500KBPS = 2,
    CAN_BITRATE_250KBPS = 3,
    CAN_BITRATE_125KBPS = 4,

    CAN_BITRATE_50KBPS = 6,
    CAN_BITRATE_20KBPS = 7,

    CAN_BITRATE_AUTO = 9,
} CANbitrate_t;

typedef enum : uint16_t {
    UART_BITRATE_9600 = 0,
    UART_BITRATE_14400 = 1,
    UART_BITRATE_19200 = 2,
    UART_BITRATE_38400 = 3,
    UART_BITRATE_57600 = 4,
    UART_BITRATE_115200 = 5,
} UARTbitrate_t;

typedef enum : int8_t {
    HM_ACTUAL_POSITION = 35,
    HM_INDEX_POSITIVE_SPEED = 34,
    HM_INDEX_NEGATIVE_SPEED = 33,
    HM_HOME_SWITCH_NEGATIVE_SPEED = 27,
    HM_HOME_SWITCH_POSITIVE_SPEED = 23,
    HM_POSITIVE_LIMIT_SWITCH = 18,
    HM_NEGATIVE_LIMIT_SWITCH = 17,
    HM_HOME_SWITCH_NEGATIVE_SPEED_AND_INDEX = 11, 
    HM_HOME_SWITCH_POSITIVE_SPEED_AND_INDEX = 7,
    HM_POSITIVE_LIMIT_SWITCH_AND_INDEX = 2,
    HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX = 1,
    HM_CURRENT_THRESHOLD_POSITIVE_SPEED_AND_INDEX = -1,
    HM_CURRENT_THRESHOLD_NEGATIVE_SPEED_AND_INDEX = -2,
    HM_CURRENT_THRESHOLD_POSITIVE_SPEED = -3,
    HM_CURRENT_THRESHOLD_NEGATIVE_SPEED = -4,
} HommingMode_t;

typedef union { 
    uint64_t value;
    struct __attribute__((packed)) {
        uint32_t position : 32;
        int32_t  velocity : 24;
        uint32_t time : 8;
    };
} IPMDataRecord_t;

#ifdef __cplusplus
}
#endif

#endif
