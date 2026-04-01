#pragma once

/*
    Thin C++ wrapper around the current C API.

    Main classes:
    - EPOS::CAN: lifecycle of the CANopen stack plus low-level CANopen helpers
      (NMT, SDO, PDO, raw CAN, subscriptions).
    - EPOS::Motor: high-level wrapper for one EPOS controller.
*/

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "canopen.h"
#include "canopen_console.h"
#include "motor.h"
#include "motor_console.h"
#include "esp_debug_helpers.h"

namespace EPOS {

namespace detail {
inline void check(esp_err_t ec)
{
#ifdef ENABLE_EPOS_CAN_EXCEPTIONS
    if (ec != ESP_OK) {
        throw ec;
    }
#else
    if (ec != ESP_OK) {
        esp_backtrace_print(32);
        abort();
    }
#endif
}
} // namespace detail

class NMTProtocol {
public:
    void enter_preoperational(uint8_t node) { detail::check(nmt_enter_preoperational(node)); }
    void reset_communication(uint8_t node) { detail::check(nmt_reset_communication(node)); }
    void reset_node(uint8_t node)          { detail::check(nmt_reset_node(node)); }
    void start_remote_node(uint8_t node)   { detail::check(nmt_start_remote_node(node)); }
    void stop_remote_node(uint8_t node)    { detail::check(nmt_stop_remote_node(node)); }

    void send(uint8_t cs, uint8_t node)    { detail::check(nmt(cs, node)); }
};

class SDOProtocol {
public:
    explicit SDOProtocol(uint32_t base = 0x600u) : _base(base) {}

    template <typename T>
    void download(uint8_t node, uint16_t index, uint8_t subindex, const T& value) const {
        static_assert(std::is_trivially_copyable_v<T>, "SDO download requires trivially copyable type");
        T tmp = value;
        detail::check(sdo_download(_base + node, index, subindex, &tmp, sizeof(tmp)));
    }

    template <typename T>
    T upload(uint8_t node, uint16_t index, uint8_t subindex) const {
        static_assert(std::is_trivially_copyable_v<T>, "SDO upload requires trivially copyable type");
        T value{};
        detail::check(sdo_upload(_base + node, index, subindex, &value));
        return value;
    }

    template <typename T>
    void upload(uint8_t node, uint16_t index, uint8_t subindex, T& value) const {
        value = upload<T>(node, index, subindex);
    }

private:
    uint32_t _base;
};

class PDOProtocol {
public:
    static constexpr uint32_t map_entry(uint16_t index, uint8_t subindex, uint8_t bit_length)
    {
        return ((uint32_t) index << 16) | ((uint32_t) subindex << 8) | bit_length;
    }

    template <typename T>
    static constexpr uint32_t map_entry(uint16_t index, uint8_t subindex)
    {
        static_assert(std::is_trivially_copyable_v<T>, "PDO mapping requires trivially copyable type");
        return map_entry(index, subindex, static_cast<uint8_t>(sizeof(T) * 8u));
    }

    void configure(uint8_t node,
                   canopen_pdo_dir_t dir,
                   uint8_t pdo_num,
                   const canopen_pdo_cfg_t& cfg) const
    {
        detail::check(canopen_pdo_configure(node, dir, pdo_num, &cfg));
    }

    void send(uint8_t node,
              uint8_t rpdo_num,
              const void* data,
              size_t len,
              uint32_t cob_id_override = 0u) const
    {
        detail::check(canopen_pdo_send(node, rpdo_num, cob_id_override, data, len));
    }

    template <typename T>
    void send(uint8_t node,
              uint8_t rpdo_num,
              const T& value,
              uint32_t cob_id_override = 0u) const
    {
        static_assert(std::is_trivially_copyable_v<T>, "PDO send requires trivially copyable type");
        send(node, rpdo_num, &value, sizeof(value), cob_id_override);
    }

    void subscribe(uint8_t node,
                   uint8_t tpdo_num,
                   canopen_handler_fn fn,
                   void* context,
                   canopen_handler_handle_t* out,
                   uint32_t cob_id_override = 0u) const
    {
        detail::check(canopen_pdo_subscribe(node, tpdo_num, cob_id_override, fn, context, out));
    }

    static void payload_clear(canopen_pdo_payload_t& p)
    {
        canopen_pdo_payload_clear(&p);
    }

    static void payload_put(canopen_pdo_payload_t& p, uint8_t v)
    {
        detail::check(canopen_pdo_payload_put_u8(&p, v));
    }

    static void payload_put(canopen_pdo_payload_t& p, uint16_t v)
    {
        detail::check(canopen_pdo_payload_put_u16(&p, v));
    }

    static void payload_put(canopen_pdo_payload_t& p, uint32_t v)
    {
        detail::check(canopen_pdo_payload_put_u32(&p, v));
    }

    static void payload_put(canopen_pdo_payload_t& p, int32_t v)
    {
        detail::check(canopen_pdo_payload_put_i32(&p, v));
    }
};

class CAN {
public:
    CAN(gpio_num_t can_tx = (gpio_num_t)DEFAULT_CAN_TX,
        gpio_num_t can_rx = (gpio_num_t)DEFAULT_CAN_RX)
        : _tx(can_tx), _rx(can_rx) {}

    void begin(bool activate_console = false,
               unsigned max_delay_ms = 3000,
               bool enable_dump_msg = false);

    void begin(const canopen_init_cfg_t& cfg, bool activate_console = false);

    void shutdown_request() { canopen_request_shutdown(); }

    void done()
    {
        canopen_request_shutdown();
        detail::check(canopen_wait_shutdown());
    }

    void wait_done()
    {
        detail::check(canopen_wait_shutdown());
    }

    template <typename T>
    void wait_until(uint32_t cobid, T& ret) const
    {
        detail::check(canopen_wait_until(cobid, &ret));
    }

    void register_handler(uint32_t cobid,
                          canopen_handler_fn fn,
                          void* context,
                          canopen_handler_handle_t* out_handle) const
    {
        detail::check(canopen_register_handler(cobid, fn, context, out_handle));
    }

    void unregister_handler(canopen_handler_handle_t handle) const
    {
        detail::check(canopen_unregister_handler(handle));
    }

    void send(const twai_message_t& msg) const
    {
        detail::check(canopen_send(&msg));
    }

    void post(const twai_message_t& msg) const
    {
        detail::check(canopen_post(&msg));
    }

    TickType_t get_max_delay_ms() const { return canopen_get_max_delay_ms(); }
    void set_max_delay_ms(unsigned delay) const { canopen_max_delay_ms(delay); }

    bool is_dump_enabled() const { return canopen_is_dump_enabled(); }
    void set_dump_enabled(bool enable) const { canopen_dump_enabled(enable); }

public:
    NMTProtocol NMT;
    SDOProtocol SDO;
    PDOProtocol PDO;

private:
    gpio_num_t _tx;
    gpio_num_t _rx;
};

class Motor {
public:
    explicit Motor(uint8_t node) { detail::check(motor_init(&_m, node)); }

    void halt() {
        detail::check(motor_halt(&_m));
    }

    int32_t get_actual_position() {
        int32_t pos{};
        detail::check(motor_get_actual_position(&_m, &pos));
        return pos;
    }

    void set_actual_position(int32_t pos) {
        detail::check(motor_set_actual_position(&_m, pos));
    }

    void homing_actual_position() {
        detail::check(motor_homing_actual_position(&_m));
    }

    void set_profile_config(const MotorProfileConfig_t& cfg) {
        auto tmp = cfg;
        detail::check(motor_set_profile_config(&_m, &tmp));
    }

    MotorProfileConfig_t get_profile_config() {
        MotorProfileConfig_t cfg{};
        detail::check(motor_get_profile_config(&_m, &cfg));
        return cfg;
    }

    void set_profile(const MotorProfile_t& p) {
        auto tmp = p;
        detail::check(motor_set_profile(&_m, &tmp));
    }

    MotorProfile_t get_profile() {
        MotorProfile_t p{};
        detail::check(motor_get_profile(&_m, &p));
        return p;
    }

    void profile_position_absolute(int32_t target_position) {
        detail::check(motor_profile_position_absolute(&_m, target_position));
    }

    void profile_position_relative(int32_t target_delta) {
        detail::check(motor_profile_position_relative(&_m, target_delta));
    }

    void profile_velocity_move(int32_t target_velocity) {
        detail::check(motor_profile_velocity_move(&_m, target_velocity));
    }

    void position_set_absolute(int32_t target_position) {
        detail::check(motor_position_set_absolute(&_m, target_position));
    }

    void current_set_current(int16_t current) {
        detail::check(motor_current_set_current(&_m, current));
    }

    void velocity_set_velocity(int32_t velocity) {
        detail::check(motor_velocity_set_velocity(&_m, velocity));
    }

    bool is_next_period(esp_cpu_cycle_count_t period) {
        return motor_is_next_period(&_m, period);
    }

    void start_period() {
        motor_start_period(&_m);
    }

    void move_to_stall(int32_t velocity, bool& done) {
        detail::check(motor_move_to_stall(&_m, velocity, &done));
    }

    void reset() {
        detail::check(motor_reset(&_m));
    }

    DeviceState_t get_device_state() {
        DeviceState_t state{};
        detail::check(motor_get_device_state(&_m, &state));
        return state;
    }

    bool is_setpoint_ack() {
        bool setpoint_ack{};
        detail::check(motor_is_setpoint_ack(&_m, &setpoint_ack));
        return setpoint_ack;
    }

    bool is_switched_on() {
        bool switched_on{};
        detail::check(motor_is_switched_on(&_m, &switched_on));
        return switched_on;
    }

    bool is_operation_enable() {
        bool operation_enable{};
        detail::check(motor_is_operation_enable(&_m, &operation_enable));
        return operation_enable;
    }

    bool is_quickstop() {
        bool quickstop{};
        detail::check(motor_is_quickstop(&_m, &quickstop));
        return quickstop;
    }

    bool is_target_reached() {
        bool target_reached{};
        detail::check(motor_is_target_reached(&_m, &target_reached));
        return target_reached;
    }

    void wait_setpoint_ack() {
        detail::check(motor_wait_setpoint_ack(&_m));
    }

    void wait_switched_on() {
        detail::check(motor_wait_switched_on(&_m));
    }

    void wait_operation_enable() {
        detail::check(motor_wait_operation_enable(&_m));
    }

    void wait_quickstop() {
        detail::check(motor_wait_quickstop(&_m));
    }

    void wait_target_reached() {
        detail::check(motor_wait_target_reached(&_m));
    }

    void wait_fault_clear() {
        detail::check(motor_wait_fault_clear(&_m));
    }

    void wait_homing_attained_or_homing_error() {
        detail::check(motor_wait_homing_attained_or_homing_error(&_m));
    }

    void activate_async_notifications(motor_async_callback cb, void* context) {
        detail::check(motor_activate_async_notifications(&_m, cb, context));
    }

    Motor_t* raw() { return &_m; }
    const Motor_t* raw() const { return &_m; }

private:
    Motor_t _m{};
};

} // namespace EPOS
