#ifndef EPOS_MOTOR_CONSOLE_H
#define EPOS_MOTOR_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "epos_motor.h"

/**
 * @brief Registra los comandos relacionados con el control del motor en la consola interactiva.
 */
void motor_register_commands(void);

#ifdef __cplusplus
}
#endif

#endif // EPOS_MOTOR_CONSOLE_H
