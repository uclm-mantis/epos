#include "epos.h"
#include "EposCAN.hh"


void EPOS::CAN::begin(bool activate_console) {
    canopen_init_cfg_t cfg = CANOPEN_INIT_DEFAULT();
    cfg.can_tx_pin = _tx;
    cfg.can_rx_pin = _rx;
    epos_initialize(&cfg);

    if (activate_console) {
        canopen_console_cfg_t console_cfg = CANOPEN_CONSOLE_DEFAULT();
        canopen_console_register_commands();
        motor_register_commands();
        canopen_console_init(&console_cfg);
    }
}
