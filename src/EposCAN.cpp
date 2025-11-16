#include "EposCAN.hh"


void EPOS::CAN::begin(bool activate_console) {
    epos_set_tx_rx_pins(_tx, _rx);
    motor_register_commands();
    epos_initialize(activate_console);
}
