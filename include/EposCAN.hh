#ifndef EPOSCAN_HH
#define EPOSCAN_HH

/*
    Thin wrapper de la API C de EPOS-CAN. Utiliza la documentación en las cabeceras
    C (epos.h y epos_motor.h) como referencia hasta que termine la documentación de
    este módulo.

    Solo hay dos clases de usuario: 
    
    - EPOS::CAN proporciona abstracciones de bajo nivel para manejo del bus CAN 
      y los protocolos NMT y SDO.  Es necesaria una instancia global de EPOS::CAN.
    
    - EPOS::Motor proporciona una abstracción de alto nivel para una controladora
      EPOS, incluyendo modos Profile Position, Profile Velocity, Position, Velocity,
      Current y algunos modos de Homing.  Es necesaria una instancia por cada 
      controladora que se vaya a usar.
*/

#include "epos.h"
#include "epos_motor.h"
#include "epos_motor_console.h"
#include "esp_debug_helpers.h"

#ifndef ENABLE_EPOS_CAN_EXCEPTIONS
#include "esp_debug_helpers.h"
#endif

namespace EPOS {

class NMTProtocol {
public:
    void enter_preoperational(uint8_t node) { nmt_enter_preoperational(node); }
    void reset_communication(uint8_t node) { nmt_reset_communication(node); }
    void reset_node(uint8_t node) { nmt_reset_node(node); }
    void start_remote_node(uint8_t node) { nmt_start_remote_node(node); }
    void stop_remote_node(uint8_t node) { nmt_stop_remote_node(node); }
};

class SDOProtocol {
public:
    SDOProtocol(uint32_t base = 0x600): _base(base) {}

    template <typename T>
    void download(uint8_t node, uint16_t index, uint8_t subindex, T value) {
        sdo_download(_base + node, index, subindex, &value, sizeof(value));
    }

    template <typename T>
    void upload(uint8_t node, uint16_t index, uint8_t subindex, T& value) {
        sdo_upload(_base + node, index, subindex, &value);
    }

private:
    uint32_t _base;
};

class CAN {
public:
    CAN(gpio_num_t can_tx = GPIO_NUM_22, gpio_num_t can_rx = GPIO_NUM_21): _tx(can_tx), _rx(can_rx) {}

    void begin(bool activate_console);
    void done(void) { epos_done(); epos_wait_done(); }

    void wait_done(void) { epos_wait_done(); }

    template <typename T>
    void wait_until(uint32_t cobid, T& ret) { epos_wait_until(cobid, &ret); }

public:
    NMTProtocol NMT;
    SDOProtocol SDO;

private:
    gpio_num_t _tx, _rx;
};

#ifdef ENABLE_EPOS_CAN_EXCEPTIONS
#   define EPOS_CAN_ERROR_IF_NOT_OK(ec) if (ec != ESP_OK) throw ec
#else
#   define EPOS_CAN_ERROR_IF_NOT_OK(ec) if (ec != ESP_OK) { esp_backtrace_print(32); abort(); }
#endif


class Motor {
public:
    Motor(uint8_t node) { motor_init(&_m, node); }

    void halt() { 
        esp_err_t ec = motor_halt(&_m); 
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    int32_t get_actual_position() { 
        int32_t pos;
        esp_err_t ec = motor_get_actual_position(&_m, &pos);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return pos;
    }

    void set_actual_position(int32_t pos) {
        esp_err_t ec = motor_set_actual_position(&_m, pos);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void homing_actual_position() {
        esp_err_t ec = motor_homing_actual_position(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void set_profile_config(const MotorProfileConfig_t& cfg) {
        esp_err_t ec = motor_set_profile_config(&_m, (MotorProfileConfig_t*)&cfg);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    MotorProfileConfig_t get_profile_config() {
        MotorProfileConfig_t cfg;
        esp_err_t ec = motor_get_profile_config(&_m, &cfg);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return cfg;
    }

    void set_profile(const MotorProfile_t& p) {
        esp_err_t ec = motor_set_profile(&_m, (MotorProfile_t*)&p);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    MotorProfile_t get_profile() {
        MotorProfile_t p;
        esp_err_t ec = motor_get_profile(&_m, &p);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return p;
    }

    void profile_position_absolute(int32_t target_position) {
        esp_err_t ec = motor_profile_position_absolute(&_m, target_position);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void profile_position_relative(int32_t target_delta) {
        esp_err_t ec = motor_profile_position_relative(&_m, target_delta);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void profile_velocity_move(int32_t target_velocity) {
        esp_err_t ec = motor_profile_velocity_move(&_m, target_velocity);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void position_set_absolute(int32_t target_position) {
        esp_err_t ec = motor_position_set_absolute(&_m, target_position);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void current_set_current(int16_t current) {
        esp_err_t ec = motor_current_set_current(&_m, current);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void velocity_set_velocity(int32_t velocity) {
        esp_err_t ec = motor_velocity_set_velocity(&_m, velocity);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    bool is_next_period(esp_cpu_cycle_count_t period) {
        return motor_is_next_period(&_m, period);
    }

    void start_period() {
        motor_is_next_period(&_m, 0);
    }

    void move_to_stall(int32_t velocity, bool& done) {
        esp_err_t ec = motor_move_to_stall(&_m, velocity, &done);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void reset() {
        esp_err_t ec = motor_reset(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    DeviceState_t get_device_state() {
        DeviceState_t state;
        esp_err_t ec = motor_get_device_state(&_m, &state);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return state;
    }

    bool is_setpoint_ack() {
        bool setpoint_ack;
        esp_err_t ec = motor_is_setpoint_ack(&_m, &setpoint_ack);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return setpoint_ack;
    }

    bool is_switched_on() {
        bool switched_on;
        esp_err_t ec = motor_is_switched_on(&_m, &switched_on);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return switched_on;
    }


    bool is_operation_enable() {
        bool operation_enable;
        esp_err_t ec = motor_is_operation_enable(&_m, &operation_enable);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return operation_enable;
    }

    bool is_quickstop() {
        bool quickstop;
        esp_err_t ec = motor_is_quickstop(&_m, &quickstop);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return quickstop;
    }

    bool is_target_reached() {
        bool target_reached;
        esp_err_t ec = motor_is_target_reached(&_m, &target_reached);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
        return target_reached;
    }

    void wait_setpoint_ack() {
        esp_err_t ec = motor_wait_setpoint_ack(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void wait_switched_on() {
        esp_err_t ec = motor_wait_switched_on(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void wait_operation_enable() {
        esp_err_t ec = motor_wait_operation_enable(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void wait_quickstop() {
        esp_err_t ec = motor_wait_quickstop(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void wait_target_reached() {
        esp_err_t ec = motor_wait_target_reached(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void wait_fault_clear() {
        esp_err_t ec = motor_wait_fault_clear(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void wait_homing_attained_or_homing_error() {
        esp_err_t ec = motor_wait_homing_attained_or_homing_error(&_m);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

    void activate_async_notifications(motor_async_callback cb, void* context) {
        esp_err_t ec = motor_activate_async_notifications(&_m, cb, context);
        EPOS_CAN_ERROR_IF_NOT_OK(ec);
    }

private:
    Motor_t _m;
};

}

#endif