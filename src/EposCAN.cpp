#include "EposCAN.hh"


void EPOS::CAN::begin(bool activate_console) {
    epos_initialize(NULL);
    if (activate_console) {
        canopen_init_cfg_t cfg = CANOPEN_INIT_DEFAULT();
        cfg.can_tx_pin = _tx;
        cfg.can_rx_pin = _rx;
        canopen_console_register_commands();
        motor_register_commands();
        canopen_console_init(&cfg);
    }
}
