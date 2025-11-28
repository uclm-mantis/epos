#include "EposCAN.hh"


void EPOS::CAN::begin(bool activate_console) {
    epos_init_cfg_t cfg = epos_init_default();
    cfg.enable_console = activate_console;
    cfg.can_tx_pin = _tx;
    cfg.can_rx_pin = _rx;
    motor_register_commands();
    epos_initialize(&cfg);
}
