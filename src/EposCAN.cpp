#include "EposCAN.hh"

namespace EPOS {

void CAN::begin(bool activate_console, unsigned max_delay_ms, bool enable_dump_msg)
{
    canopen_init_cfg_t cfg = canopen_init_default();
    cfg.can_tx_pin = _tx;
    cfg.can_rx_pin = _rx;
    cfg.max_delay_ms = max_delay_ms;
    cfg.enable_dump_msg = enable_dump_msg;
    begin(cfg, activate_console);
}

void CAN::begin(const canopen_init_cfg_t& cfg, bool activate_console)
{
    detail::check(canopen_initialize(&cfg));

    if (activate_console) {
        canopen_console_cfg_t console_cfg = CANOPEN_CONSOLE_DEFAULT();
        canopen_console_register_commands();
        motor_register_commands();
        canopen_console_init(&console_cfg);
    }
}

} // namespace EPOS
