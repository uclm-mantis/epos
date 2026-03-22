#pragma once

#include <stdint.h>
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registra los comandos para el control de motores CiA 402 en la consola interactiva.
 */
void motor_register_commands(void);

#ifdef __cplusplus
}
#endif
