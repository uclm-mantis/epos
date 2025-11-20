#pragma once

#include <stdint.h>
#include "epos_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registra los comandos relacionados con el control del motor en la consola interactiva.
 */
void motor_register_commands(void);

#ifdef __cplusplus
}
#endif
