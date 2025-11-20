#pragma once

#include "epos_types.h"
#include "esp_cpu.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Estructura principal que representa un motor
    
    Contiene toda la información necesaria para controlar y monitorizar
    un motor EPOS, incluyendo su configuración, estado actual y límites
    de movimiento.
*/
typedef struct {
    uint8_t node;       /*!< Identificador del motor en la red CANopen */

    OperationMode_t mode; /*!< Modo de movimiento actual */
    uint8_t substate;   /*!< Subestado dentro del modo actual */

    int32_t limits[2];  /*!< Límites de movimiento [inferior, superior] */
    int32_t position;   /*!< Última posición conocida del motor */

    esp_cpu_cycle_count_t sample_period; /*!< Periodo de muestreo en ciclos de CPU */
    esp_cpu_cycle_count_t last;          /*!< Último instante de muestreo */
    int16_t threshold;                   /*!< Umbral para detección de límite (en mA) */
} Motor_t;

/*! @brief Parámetros dinámicos del perfil de movimiento */
typedef struct {
    uint32_t velocity;     /*!< Velocidad objetivo */
    uint32_t acceleration; /*!< Aceleración para alcanzar la velocidad objetivo */
    uint32_t deceleration; /*!< Deceleración para reducir la velocidad */
    /*! @brief Tipo de perfil de movimiento */
    enum : int16_t {
        PPM_TRAPEZOIDAL, /*!< Perfil trapezoidal (velocidad constante) */
        PPM_SINUSOIDAL,  /*!< Perfil sinusoidal (aceleración suave) */
    } type;
} MotorProfile_t;

/*! @brief Configuración del perfil de movimiento
    
    Define los límites y parámetros de control para el
    movimiento del motor.
*/
typedef struct {
    uint32_t position_window;         /*!< Ventana de posición considerada como objetivo alcanzado */
    uint16_t position_window_time;    /*!< Tiempo mínimo dentro de la ventana de posición */
    int32_t  min_position_limit;      /*!< Límite inferior de posición */
    int32_t  max_position_limit;      /*!< Límite superior de posición */
    uint32_t max_velocity;            /*!< Velocidad máxima permitida */
    uint32_t quickstop_deceleration;  /*!< Deceleración para parada de emergencia */
    uint32_t max_acceleration;        /*!< Aceleración máxima permitida */
} MotorProfileConfig_t;

/*! @brief Estados posibles del dispositivo según tabla 3-4 de la especificación */
typedef enum {
    DEVICE_START,                         /*!< Dispositivo iniciando */
    DEVICE_NOT_READY_TO_SWITCH_ON,       /*!< No preparado para encender */
    DEVICE_SWITCH_ON_DISABLED,           /*!< Encendido deshabilitado */
    DEVICE_READY_TO_SWITCH_ON,           /*!< Preparado para encender */
    DEVICE_SWITCHED_ON,                  /*!< Encendido */
    DEVICE_REFRESH,                      /*!< Actualizando estado */
    DEVICE_MEASURE_INIT,                /*!< Inicializando medidas */
    DEVICE_OPERATION_ENABLE,            /*!< Operación habilitada */
    DEVICE_QUICKSTOP_ACTIVE,           /*!< Parada rápida activa */
    DEVICE_FAULT_REACTION_ACTIVE_DISABLED, /*!< Reacción a fallo activa (deshabilitado) */
    DEVICE_FAULT_REACTION_ACTIVE_ENABLED,  /*!< Reacción a fallo activa (habilitado) */
    DEVICE_FAULT,                        /*!< Estado de fallo */
} DeviceState_t;

/*! @brief Constructor de la abstracción @a Motor_t

    Construye un nuevo objeto @a Motor_t en el estado inicial. Reserva memoria
    con @a malloc e inicializa los valores con @a motor_init.

    @see motor_init
    @param node Identificador del nodo.
    @return Puntero a nuevo objeto @a Motor_t.
*/
Motor_t* motor_new(uint8_t node);

/*! @brief Inicialización de abstracción @a Motor_t

    Inicializa la memoria correspondiente a un @a Motor_t a valores iniciales
    conocidos. En principio solo es necesario inicializar el identificador,
    los límites, que se configuran con un centinela que implica desconocido, y
    el valor del estado, que determina las operaciones que son posibles en cada
    momento.

    @param[out] self Puntero a objeto @a Motor_t que se inicializará.
    @param[in]  node Identificador del nodo.
    @return @a ESP_OK si no hay error. 
*/
esp_err_t motor_init(Motor_t* self, uint8_t node);

/*! @brief Detiene el movimiento del motor
    
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_halt(Motor_t* self);

/*! @brief Obtiene la posición absoluta actual.
    
    @param[in]  self Puntero al objeto Motor_t
    @param[out] pos  Puntero a posición absoluta
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_get_actual_position(Motor_t* self, int32_t* pos);

/*! @brief Fija la posición actual usando modo homing.
    
    @param[in]  self Puntero al objeto Motor_t
    @param[out] pos  Posición absoluta
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_set_actual_position(Motor_t* self, int32_t pos);

/*! @brief Realiza el homing del motor usando la posición actual
    
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_homing_actual_position(Motor_t* self);

/*! @brief Configura los parámetros del perfil de movimiento del motor
    
    @param[in] self Puntero al objeto Motor_t
    @param[in] cfg  Configuración del perfil a establecer
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_set_profile_config(Motor_t* self, MotorProfileConfig_t* cfg);

/*! @brief Obtiene los parámetros actuales del perfil de movimiento del motor
    
    @param[in]  self Puntero al objeto Motor_t
    @param[out] cfg  Estructura donde se almacenará la configuración actual
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_get_profile_config(Motor_t* self, MotorProfileConfig_t* cfg);

/*! @brief Establece los parámetros dinámicos del perfil de movimiento
    
    @param[in] self Puntero al objeto Motor_t
    @param[in] p    Parámetros del perfil a establecer
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_set_profile(Motor_t* self, MotorProfile_t* p);

/*! @brief Obtiene los parámetros dinámicos actuales del perfil de movimiento
    
    @param[in]  self Puntero al objeto Motor_t
    @param[out] p    Estructura donde se almacenarán los parámetros actuales
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_get_profile(Motor_t* self, MotorProfile_t* p);

/*! @brief Mueve el motor a una posición absoluta usando el modo perfil de posición
    
    @param[in] self            Puntero al objeto Motor_t
    @param[in] target_position Posición objetivo absoluta
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_profile_position_absolute(Motor_t* self, int32_t target_position);

/*! @brief Mueve el motor una distancia relativa usando el modo perfil de posición
    
    @param[in] self         Puntero al objeto Motor_t
    @param[in] target_delta Distancia relativa a mover
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_profile_position_relative(Motor_t* self, int32_t target_delta);

/*! @brief Mueve el motor a una velocidad objetivo usando el modo perfil de velocidad
    
    @param[in] self            Puntero al objeto Motor_t
    @param[in] target_velocity Velocidad objetivo
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_profile_velocity_move(Motor_t* self, int32_t target_velocity);

/*! @brief Mueve el motor a una posición absoluta usando el modo de posición
    
    @param[in] self            Puntero al objeto Motor_t
    @param[in] target_position Posición objetivo absoluta
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_position_set_absolute(Motor_t* self, int32_t target_position);

/*! @brief Fija la corriente del motor usando el modo de control de corriente
    
    @param[in] self     Puntero al objeto Motor_t
    @param[in] current  Corriente objetivo
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_current_set_current(Motor_t* self, int16_t current);

/*! @brief Fija la velocidad del motor usando el modo de control de velocidad
    
    @param[in] self      Puntero al objeto Motor_t
    @param[in] velocity  Velocidad objetivo
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_velocity_set_velocity(Motor_t* self, int32_t velocity);

/*! @brief Comprueba si ha transcurrido un periodo desde la última llamada
    
    @param[in] self   Puntero al objeto Motor_t
    @param[in] period Periodo en ciclos de CPU
    @return true si ha transcurrido el periodo, false en caso contrario
*/
bool motor_is_next_period(Motor_t* self, esp_cpu_cycle_count_t period);

/*! @brief Macro que inicia un nuevo periodo de muestreo */
#define motor_start_period(self) motor_is_next_period(self, 0)

/*! @brief Mueve el motor hasta el límite físico detectado por un pico de corriente
   
    Esta función utiliza el modo con perfil de velocidad para detectar el momento
    en que la corriente pasa a ser significativamente mayor. No es bloqueante,
    debe ejecutarse repetidamente hasta que @ref done sea true. En ese momento
    el valor de @a self->limits se actualizará. Si la velocidad es negativa se
    actualiza @a self->limits[0] y en caso contrario @a self->limits[1].
    
    @param[in]  self     Motor a mover
    @param[in]  velocity Velocidad de movimiento. El signo indica el sentido de movimiento
    @param[inout] done     Variable donde notifica el comienzo o la terminación del proceso
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_move_to_stall(Motor_t* self, int32_t velocity, bool* done);

/*! @brief Reinicia el motor y lo configura en estado operativo
    
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_reset(Motor_t* self);

/*! @brief Obtiene el estado actual del dispositivo
    
    @param[in]  self  Puntero al objeto Motor_t
    @param[out] state Estado actual del dispositivo
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_get_device_state(Motor_t* self, DeviceState_t* state);

/*! @brief Comprueba si se ha recibido el acknowledgment del setpoint
    @param[in] self Puntero al objeto Motor_t
    @param[out] setpoint_ack Puntero a variable booleana que indica si se ha recibido el acknowledgment
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_is_setpoint_ack(Motor_t* self, bool* setpoint_ack);

/*! @brief Comprueba si el motor está encendido
    @param[in] self Puntero al objeto Motor_t
    @param[out] switched_on Puntero a variable booleana que indica si el motor está encendido
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_is_switched_on(Motor_t* self, bool* switched_on);

/*! @brief Comprueba si la operación está habilitada
    @param[in] self Puntero al objeto Motor_t
    @param[out] operation_enable Puntero a variable booleana que indica si la operación está habilitada
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_is_operation_enable(Motor_t* self, bool* operation_enable);

/*! @brief Comprueba si el motor está en modo de parada rápida
    @param[in] self Puntero al objeto Motor_t
    @param[out] quickstop Puntero a variable booleana que indica si el motor está en modo de parada rápida
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_is_quickstop(Motor_t* self, bool* quickstop);

/*! @brief Comprueba si se ha alcanzado el objetivo de movimiento
    @param[in] self Puntero al objeto Motor_t
    @param[out] target_reached Puntero a variable booleana que indica si se ha alcanzado el objetivo
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_is_target_reached(Motor_t* self, bool* target_reached);

/*! @brief Espera hasta que se recibe el acknowledgment del setpoint
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_setpoint_ack(Motor_t* self);

/*! @brief Espera hasta que el motor esté encendido
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_switched_on(Motor_t* self);

/*! @brief Espera hasta que la operación esté habilitada
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_operation_enable(Motor_t* self);

/*! @brief Espera hasta que el motor esté en modo de parada rápida
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_quickstop(Motor_t* self);

/*! @brief Espera hasta que se alcance el objetivo de movimiento
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_target_reached(Motor_t* self);

/*! @brief Espera hasta que se limpie un fallo en el motor
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_fault_clear(Motor_t* self);

/*! @brief Espera hasta que se complete el proceso de homing o se produzca un error
    @param[in] self Puntero al objeto Motor_t
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_wait_homing_attained_or_homing_error(Motor_t* self);


typedef struct __attribute__((packed)) {
    int32_t position;
    StatusWord_t status;
    int16_t current;
} MotorAsyncNotification_t;

typedef void (*motor_async_callback)(uint32_t cobid, MotorAsyncNotification_t* data, void* context);

/*! @brief Activa el envío de notificaciones asíncronas (PDO).
    @param[in] self      Puntero al objeto Motor_t
    @param[in] cb        Callback para notificaciones asíncronas
    @param[in] context   Puntero pasado al callback
    @return ESP_OK si no hay error, otro código en caso contrario
*/
esp_err_t motor_activate_async_notifications(Motor_t* self, motor_async_callback cb, void* context);


// DEBUG
void print_status(OperationMode_t mode, StatusWord_t s);
void print_controlword(OperationMode_t mode, ControlWord_t c);

#ifdef __cplusplus
}
#endif
