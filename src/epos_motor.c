#include "epos_motor.h"
#include "epos_types.h"
#include "epos.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "moving_average.h"

/*!
    TODO: Actualmente tenemos implementado:
    - Homing usando la posición actual. Hay muchos más métodos.
    - Modo profile velocity y modo profile position
    - Current mode, velocity mode y position mode

    Quedan sin implementar, pero deberían tratarse en trabajo futuro:
        - Interpolated position mode. Este es el más interesante de 
          lo que se queda sin hacer. Me da tanta pena que intentaré 
          darle una vuelta.
        - Homing mode. El modo de homing con la posición actual es
          muy limitado. Deberían implementarse los demás métodos de 
          homing.

        - Master encoder mode
        - Step direction mode
        - Diagnostic mode

        De estos 3 últimos no se nada, al menos habría que tener una idea de 
        para qué sirven y si se ve interesante añadirlo para trabajo futuro. 
 */

/*! @brief Cadena que identifica los mensajes de registro de este módulo */
static const char* const TAG = "MOTOR";


Motor_t* 
motor_new(uint8_t node)
{
    Motor_t* self = (Motor_t*) malloc(sizeof(Motor_t));
    ESP_ERROR_CHECK_WITHOUT_ABORT(motor_init(self, node));
    return self;
}


esp_err_t
motor_init(Motor_t* self, uint8_t node)
{
    self->node = node;
    self->limits[0] = -1;
    self->limits[1] = -1;
    self->mode = UNKNOWN_MODE;
    self->threshold = 6000;
    self->sample_period = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 10000;
    ESP_RETURN_ON_ERROR(get_position_actual_value(node, &self->position), TAG, "Unable to get actual position");
    return ESP_OK;
}

/*! @brief Comprueba si se activa un bit en la palabra de estado.

    Algunos métodos requieren una espera activa hasta que un bit o un conjunto de 
    bits de la palabra de estado alcanzan cierto valor. Con esta macro se puede 
    automatizar la generación de estas funciones usando una expresión a comprobar,
    una expresión para el valor de retorno, y un sufijo para la función generada.

    @see @a MOTOR_WAIT_STATUS_BIT

    @param bit       Nombre de lo que espera. La función se llamará `motor_is_` seguido de @a bit. 
    @param member    Nombre del campo de la estructura @a StatusWord_t . 
*/
#define MOTOR_IS_STATUS_BIT(bit,member) \
esp_err_t motor_is_##bit(Motor_t* self, bool* bit) { \
    StatusWord_t status; \
    ESP_RETURN_ON_ERROR(get_statusword(self->node, &status.value), TAG, "Unable to read status word"); \
    *bit = status.member; \
    return ESP_OK; \
}

#define MOTOR_IS_STATUS_BIT_SIMPLE(bit) MOTOR_IS_STATUS_BIT(bit, bit)

/*! @brief Define @a motor_is_setpoint_ack. */
MOTOR_IS_STATUS_BIT(setpoint_ack, ppm.setpoint_ack)
/*! @brief Define @a motor_is_switched_on. */
MOTOR_IS_STATUS_BIT_SIMPLE(switched_on)
/*! @brief Define @a motor_is_operation_enable. */
MOTOR_IS_STATUS_BIT_SIMPLE(operation_enable)
/*! @brief Define @a motor_is_quickstop. */
MOTOR_IS_STATUS_BIT_SIMPLE(quickstop)
/*! @brief Define @a motor_is_target_reached. */
MOTOR_IS_STATUS_BIT_SIMPLE(target_reached)

/*! @brief Espera hasta que se cumple cierta condición en la palabra de estado.

    Algunos métodos requieren una espera activa hasta que un bit o un conjunto de 
    bits de la palabra de estado alcanzan cierto valor. Con esta macro se puede 
    automatizar la generación de estas funciones usando una expresión a comprobar,
    una expresión para el valor de retorno, y un sufijo para la función generada.

    @see @a MOTOR_WAIT_STATUS_BIT
    @see @a MOTOR_WAIT_STATUS_BIT_CLEAR
    @see @a MOTOR_WAIT_STATUS_ANY_2BIT

    @param name       Nombre de lo que espera. La función se llamará `motor_wait_` seguido de @a name y seguido de @a suffix. 
    @param check_expr Expresión a comprobar. El bucle termina cuando sea cierta.
    @param ret_expr   Valor a devolver cuando se cumple la condición. 
    @param suffix     Sufijo para añadir al nombre de la función (puede ser vacío).
*/
#define MOTOR_WAIT_STATUS(name, check_expr, ret_expr, suffix) \
esp_err_t motor_wait_##name##suffix(Motor_t* self) { \
    StatusWord_t status; \
    do { ESP_RETURN_ON_ERROR(get_statusword(self->node, &status.value), TAG, "Unable to read status word"); vTaskDelay(1); \
    } while (!(check_expr)); \
    return ret_expr; \
}

/*! @brief Espera hasta que se activa un bit en la palabra de estado.

    Esta macro automatiza la generación de funciones de la forma `motor_wait_##bit(m)` que 
    espera hasta que un bit determinado se activa en la palabra de estado. 

    @see StatusWord_t

    @param bit     Nombre de lo que espera. La función se llamará `motor_wait_` seguido de @a bit. 
    @param member  Nombre del campo dentro de la palabra de estado.

    @return @a ESP_OK si no hay error.
*/
#define MOTOR_WAIT_STATUS_BIT(bit, member) \
    MOTOR_WAIT_STATUS(bit, (status.member), ESP_OK,)

/*! @brief Espera hasta que se desactiva un bit en la palabra de estado.

    Esta macro automatiza la generación de funciones de la forma `motor_wait_##bit##_clear(m)` que 
    espera hasta que un bit determinado se desactiva en la palabra de estado. 

    @see StatusWord_t

    @param bit     Nombre de lo que espera. La función se llamará `motor_wait_` seguido de @a bit seguido de `_clear`. 
    @param member  Nombre del campo dentro de la palabra de estado.

    @return @a ESP_OK si no hay error.
*/
#define MOTOR_WAIT_STATUS_BIT_CLEAR(bit, member) \
    MOTOR_WAIT_STATUS(bit, (!status.member), ESP_OK, _clear)

#define MOTOR_WAIT_STATUS_BIT_SIMPLE(bit) MOTOR_WAIT_STATUS_BIT(bit, bit)
#define MOTOR_WAIT_STATUS_BIT_CLEAR_SIMPLE(bit) MOTOR_WAIT_STATUS_BIT_CLEAR(bit, bit)

/*! @brief Espera hasta que se activa un bit de entre dos posibles en la palabra de estado.

    Esta macro automatiza la generación de funciones de la forma `motor_wait_##bits##(m)` que 
    espera hasta que member1 o member2 se activan en la palabra de estado. 

    @see StatusWord_t

    @param bits     Sufijo para el bombre de la función generada.
    @param member1  Nombre de un campo dentro de la palabra de estado.
    @param member2  Nombre de otro campo dentro de la palabra de estado.

    @return @a ESP_OK si se activa @a bit1, @a ESP_FAIL si se activa @a bit2.
*/
#define MOTOR_WAIT_STATUS_ANY_2BIT(bits, member1, member2) \
    MOTOR_WAIT_STATUS(bits, (status.member1 || status.member2), (status.member1 ? ESP_OK : ESP_FAIL),)

/*! @brief Define @a motor_wait_setpoint_ack. */
MOTOR_WAIT_STATUS_BIT(setpoint_ack, ppm.setpoint_ack)
/*! @brief Define @a motor_wait_switched_on. */
MOTOR_WAIT_STATUS_BIT_SIMPLE(switched_on)
/*! @brief Define @a motor_wait_operation_enable. */
MOTOR_WAIT_STATUS_BIT_SIMPLE(operation_enable)
/*! @brief Define @a motor_wait_quickstop. */
MOTOR_WAIT_STATUS_BIT_SIMPLE(quickstop)
/*! @brief Define @a motor_wait_target_reached. */
MOTOR_WAIT_STATUS_BIT_SIMPLE(target_reached)
/*! @brief Define @a motor_wait_fault_clear. */
MOTOR_WAIT_STATUS_BIT_CLEAR_SIMPLE(fault)
/*! @brief Define @a motor_wait_homing_attained_or_homing_error. */
MOTOR_WAIT_STATUS_ANY_2BIT(homing_attained_or_homing_error, hmm.homing_attained, hmm.homing_error)


esp_err_t 
motor_get_actual_position(Motor_t* self, int32_t* pos)
{
    ESP_RETURN_ON_ERROR(get_position_actual_value(self->node, pos), TAG, "Unable to get position");
    return ESP_OK;
}


esp_err_t
motor_homing_actual_position(Motor_t* self) 
{
    return motor_set_actual_position(self, 0);
}


esp_err_t
motor_set_actual_position(Motor_t* self, int32_t pos) 
{
    self->mode = HOMING_MODE; 
    ControlWord_t cw0 = { .hmm = {.all1 = 0xf} };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw0.value), TAG, "Unable to reset CW");
    ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, HOMING_MODE), TAG, "Unable to set homing mode");
    ESP_RETURN_ON_ERROR(set_homing_method(self->node, HM_ACTUAL_POSITION), TAG, "Unable to set homing method");
    ESP_RETURN_ON_ERROR(set_home_position(self->node, pos), TAG, "Unable to set home position");
    ControlWord_t cw = { .hmm = {.all1 = 0xf, .homing_start = 1} };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to start homing");
    return ESP_OK;
}

esp_err_t 
motor_set_profile_config(Motor_t* self, MotorProfileConfig_t* cfg)
{
    ESP_RETURN_ON_ERROR(set_position_window(self->node, cfg->position_window), TAG, "Unable to set PPM window");
    ESP_RETURN_ON_ERROR(set_position_window_time(self->node, cfg->position_window_time), TAG, "Unable to set PPM window time");
    ESP_RETURN_ON_ERROR(set_min_position_limit(self->node, cfg->min_position_limit), TAG, "Unable to set PPM min limit");
    ESP_RETURN_ON_ERROR(set_max_position_limit(self->node, cfg->max_position_limit), TAG, "Unable to set PPM max limit");
    ESP_RETURN_ON_ERROR(set_max_profile_velocity(self->node, cfg->max_velocity), TAG, "Unable to set PPM max velocity");
    ESP_RETURN_ON_ERROR(set_quickstop_deceleration(self->node, cfg->quickstop_deceleration), TAG, "Unable to set PPM quickstop deceleration");
    ESP_RETURN_ON_ERROR(set_max_acceleration(self->node, cfg->max_acceleration), TAG, "Unable to set PPM max acceleration");
    return ESP_OK;
}

esp_err_t 
motor_get_profile_config(Motor_t* self, MotorProfileConfig_t* cfg)
{
    ESP_RETURN_ON_ERROR(get_position_window(self->node, &cfg->position_window), TAG, "Unable to get PPM window");
    ESP_RETURN_ON_ERROR(get_position_window_time(self->node, &cfg->position_window_time), TAG, "Unable to get PPM window time");
    ESP_RETURN_ON_ERROR(get_min_position_limit(self->node, &cfg->min_position_limit), TAG, "Unable to get PPM min limit");
    ESP_RETURN_ON_ERROR(get_max_position_limit(self->node, &cfg->max_position_limit), TAG, "Unable to get PPM max limit");
    ESP_RETURN_ON_ERROR(get_max_profile_velocity(self->node, &cfg->max_velocity), TAG, "Unable to get PPM max velocity");
    ESP_RETURN_ON_ERROR(get_quickstop_deceleration(self->node, &cfg->quickstop_deceleration), TAG, "Unable to get PPM quickstop deceleration");
    ESP_RETURN_ON_ERROR(get_max_acceleration(self->node, &cfg->max_acceleration), TAG, "Unable to get PPM max acceleration");
    return ESP_OK;
}


esp_err_t 
motor_set_profile(Motor_t* self, MotorProfile_t* p)
{
    ESP_RETURN_ON_ERROR(set_profile_velocity(self->node, p->velocity), TAG, "Unable to set PPM velocity");
    ESP_RETURN_ON_ERROR(set_profile_acceleration(self->node, p->acceleration), TAG, "Unable to set PPM acceleration");
    ESP_RETURN_ON_ERROR(set_profile_deceleration(self->node, p->deceleration), TAG, "Unable to set PPM deceleration");
    ESP_RETURN_ON_ERROR(set_motion_profile_type(self->node, p->type), TAG, "Unable to set PPM type");
    return ESP_OK;
}


esp_err_t 
motor_get_profile(Motor_t* self, MotorProfile_t* p)
{
    ESP_RETURN_ON_ERROR(get_profile_velocity(self->node, &p->velocity), TAG, "Unable to get PPM velocity");
    ESP_RETURN_ON_ERROR(get_profile_acceleration(self->node, &p->acceleration), TAG, "Unable to get PPM acceleration");
    ESP_RETURN_ON_ERROR(get_profile_deceleration(self->node, &p->deceleration), TAG, "Unable to get PPM deceleration");
    ESP_RETURN_ON_ERROR(get_motion_profile_type(self->node, &p->type), TAG, "Unable to get PPM type");
    return ESP_OK;
}


esp_err_t 
motor_profile_position_absolute(Motor_t* self, int32_t target_position)
{
    if (self->mode != PROFILE_POSITION_MODE) {
        ControlWord_t cw0 = { .ppm = {.all1 = 0xf } };
        ESP_RETURN_ON_ERROR(set_controlword(self->node, cw0.value), TAG, "Unable to reset CW");
        ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, PROFILE_POSITION_MODE), TAG, "Unable to set PPM");
        self->mode = PROFILE_POSITION_MODE;
    }
    ESP_RETURN_ON_ERROR(set_target_position(self->node, target_position), TAG, "Unable to set target position %ld", target_position);

    /* ch_set_immediate interrumpe el movimiento en curso si lo hay*/
    ControlWord_t cw = { .ppm = { .all1 = 0xf, .new_setpoint = 1, .ch_set_immediate = 1 } };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to trigger relative set point");
    ESP_RETURN_ON_ERROR(motor_wait_setpoint_ack(self), TAG, "Failed to receive ack");
    return ESP_OK;
}


esp_err_t 
motor_profile_position_relative(Motor_t* self, int32_t target_delta)
{
    if (self->mode != PROFILE_POSITION_MODE) {
        ControlWord_t cw0 = { .ppm = {.all1 = 0xf } };
        ESP_RETURN_ON_ERROR(set_controlword(self->node, cw0.value), TAG, "Unable to reset CW");
        ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, PROFILE_POSITION_MODE), TAG, "Unable to set PPM");
        self->mode = PROFILE_POSITION_MODE;
    }
    ESP_RETURN_ON_ERROR(set_target_position(self->node, target_delta), TAG, "Unable to set target delta %ld", target_delta);

    /* ch_set_immediate interrumpe el movimiento en curso si lo hay*/
    ControlWord_t cw = { .ppm = {.all1 = 0xf, .new_setpoint = 1, .ch_set_immediate = 1, .relative = 1 } };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to trigger relative set point");
    ESP_RETURN_ON_ERROR(motor_wait_setpoint_ack(self), TAG, "Failed to receive ack");
    return ESP_OK;
}


esp_err_t 
motor_halt(Motor_t* self)
{
    ControlWord_t cw = { .enable_voltage = 1, .quickstop = 1, .switch_on = 1, .enable_operation = 1, .halt = 1 };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to halt in PPM");
    return ESP_OK;
}


esp_err_t 
motor_profile_velocity_move(Motor_t* self, int32_t target_velocity)
{
    ESP_RETURN_ON_ERROR(set_target_velocity(self->node, target_velocity), TAG, "Unable to set target velocity %ld", target_velocity);
    if (self->mode != PROFILE_VELOCITY_MODE) {
        ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, PROFILE_VELOCITY_MODE), TAG, "Unable to set PPM");
        self->mode = PROFILE_VELOCITY_MODE;
    }
    // quita halt si estuviera
    ControlWord_t cw = { .enable_voltage = 1, .quickstop = 1, .switch_on = 1, .enable_operation = 1 };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to write CW");
    return ESP_OK;
}


esp_err_t
motor_interpolated_data(Motor_t* self, uint32_t position)
{

    return ESP_OK;
}

esp_err_t
motor_interpolated_active(Motor_t* self, bool active)
{
    return ESP_OK;
}

esp_err_t 
motor_position_set_absolute(Motor_t* self, int32_t target_position)
{
    ESP_RETURN_ON_ERROR(set_position_mode_setting_value(self->node, target_position), TAG, "Unable to set position %ld", target_position);
    if (self->mode != POSITION_MODE) {
        ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, POSITION_MODE), TAG, "Unable to set position mode");
        self->mode = POSITION_MODE;
    }
    // quita halt si estuviera
    ControlWord_t cw = { .enable_voltage = 1, .quickstop = 1, .switch_on = 1, .enable_operation = 1 };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to write CW");
    return ESP_OK;
}

esp_err_t 
motor_current_set_current(Motor_t* self, int16_t current)
{
    ESP_RETURN_ON_ERROR(set_current_mode_setting_value(self->node, current), TAG, "Unable to set current to %d", current);
    if (self->mode != CURRENT_MODE) {
        ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, CURRENT_MODE), TAG, "Unable to set position mode");
        self->mode = CURRENT_MODE;
    }
    // quita halt si estuviera
    ControlWord_t cw = { .enable_voltage = 1, .quickstop = 1, .switch_on = 1, .enable_operation = 1 };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to write CW");
    return ESP_OK;
}

esp_err_t 
motor_velocity_set_velocity(Motor_t* self, int32_t velocity)
{
    ESP_RETURN_ON_ERROR(set_velocity_mode_setting_value(self->node, velocity), TAG, "Unable to set velocity %ld", velocity);
    if (self->mode != VELOCITY_MODE) {
        ESP_RETURN_ON_ERROR(set_modes_of_operation(self->node, VELOCITY_MODE), TAG, "Unable to set position mode");
        self->mode = VELOCITY_MODE;
    }
    // quita halt si estuviera
    ControlWord_t cw = { .enable_voltage = 1, .quickstop = 1, .switch_on = 1, .enable_operation = 1 };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to write CW");
    return ESP_OK;
}

bool 
motor_is_next_period(Motor_t* self, esp_cpu_cycle_count_t period)
{
    esp_cpu_cycle_count_t now = esp_cpu_get_cycle_count();
    if (now - self->last > period) { 
        self->last = now; 
        return true;
    } 
    return false;
}


/*
    motor_move_to_stall no para el motor, solo activa done para que el cliente haga lo
    que sea.  Esto es necesario para que los motores que se mueven de forma sincronizada no
    se desincronicen por esta causa.  Por ejemplo, en el panel hay que esperar a que los dos
    lleguen al final para cortar la corriente.

    Nótese que ahora el umbral se comprueba una sola vez, se supone que es un salto grande 
    respecto a la media.

    done se usa para comunicación bidireccional. Si está a true es que estamos empezando
    el move to stall.  Por tanto se usaría así:

    bool done = true;
    do {
        motor_move_to_stall(m, v, &done);
    } while(!done);
 */
esp_err_t 
motor_move_to_stall(Motor_t* self, int32_t velocity, bool* done)
{
    if (*done || self->mode != PROFILE_VELOCITY_MODE) {
        *done = false;
        motor_start_period(self);
        ESP_LOGI(TAG, "%d: move_to_stall speed %ld", self->node, velocity);
        ESP_RETURN_ON_ERROR(motor_profile_velocity_move(self, velocity), TAG, "Unable to run velocity operation");
    }

    if (motor_is_next_period(self, self->sample_period)) {
        int16_t current;
        ESP_RETURN_ON_ERROR(get_current_actual_value_averaged(self->node, &current), TAG, "Unable to read current");
        *done = (abs(current) > self->threshold);
        if (*done)
            ESP_LOGI(TAG, "%d: done current=%hd", self->node, current);
    }
    return ESP_OK;
}


esp_err_t 
motor_reset(Motor_t* self)
{
    ESP_RETURN_ON_ERROR(nmt_reset_node(self->node), TAG, "Unable to NMT reset");
    ESP_RETURN_ON_ERROR(epos_wait_until(0x700 + self->node, NULL), TAG, "Did not receive NMT bootup");

    DeviceState_t state;
    ESP_RETURN_ON_ERROR(motor_get_device_state(self, &state), TAG, "Unable to get initial state");
    if (state == DEVICE_FAULT) {
        ControlWord_t cw = { .fault_reset = 1 };
        ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to set control word");
        ESP_RETURN_ON_ERROR(motor_wait_fault_clear(self), TAG, "Failed waiting for fault clear");
    }

    ControlWord_t cw = { .enable_voltage = 1, .quickstop = 1 };
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to set control word");
    ESP_RETURN_ON_ERROR(motor_wait_quickstop(self), TAG, "Failed waiting for quickstop");

    cw.switch_on = 1;
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to set control word");
    ESP_RETURN_ON_ERROR(motor_wait_switched_on(self), TAG, "Failed waiting for switch on");

    cw.enable_operation = 1;
    ESP_RETURN_ON_ERROR(set_controlword(self->node, cw.value), TAG, "Unable to set control word");
    ESP_RETURN_ON_ERROR(motor_wait_operation_enable(self), TAG, "Failed waiting for operation enable");

    return ESP_OK;
}

esp_err_t 
motor_activate_async_notifications(Motor_t* self, motor_async_callback cb, void* context)
{
    ESP_RETURN_ON_ERROR(nmt_enter_preoperational(self->node), TAG, "Unable to -> preoperational");
    ESP_RETURN_ON_ERROR(set_cob_id_used_by_txpdo_1(self->node, 0x40000180 + self->node), TAG, "Unable to wr txPDO1 COBid");
    ESP_RETURN_ON_ERROR(set_transmission_type_in_txpdo_1(self->node, 255), TAG, "Unable to wr txPDO1 type");
    ESP_RETURN_ON_ERROR(set_inhibit_time_in_txpdo_1(self->node, 100), TAG, "Unable to wr txPDO1 time");
    ESP_RETURN_ON_ERROR(set_number_of_mapped_application_objects_in_txpdo_1(self->node, 3), TAG, "Unable to wr txPDO1 n_obj");
    ESP_RETURN_ON_ERROR(set_1st_mapped_object_in_txpdo_1(self->node, txpdo_position_actual_value), TAG, "Unable to wr txPDO1 obj1");
    ESP_RETURN_ON_ERROR(set_2nd_mapped_object_in_txpdo_1(self->node, txpdo_statusword), TAG, "Unable to wr txPDO1 obj2");
    ESP_RETURN_ON_ERROR(set_3rd_mapped_object_in_txpdo_1(self->node, txpdo_current_actual_value_averaged), TAG, "Unable to wr txPDO1 obj3");
    ESP_RETURN_ON_ERROR(epos_register_canopen_handler(0x180 + self->node, (canopen_handler_fn)cb, context), TAG, "Unable to setup txPDO1 cb");
    ESP_RETURN_ON_ERROR(nmt_start_remote_node(self->node), TAG, "Unable to -> operational");
    return ESP_OK;
}


esp_err_t 
motor_get_device_state(Motor_t* self, DeviceState_t* state)
{
    // state of drive as of Table 3-4 of Firmware Specification
    const uint16_t mask = 0b0100000101111111;
    static struct {
        uint16_t value;
        DeviceState_t state;
    } patterns[] = {
        { 0b0000000000000000, DEVICE_START },
        { 0b0000000100000000, DEVICE_NOT_READY_TO_SWITCH_ON },
        { 0b0000000101000000, DEVICE_SWITCH_ON_DISABLED },
        { 0b0000000100100001, DEVICE_READY_TO_SWITCH_ON },
        { 0b0000000100100011, DEVICE_SWITCHED_ON },
        { 0b1000000100100011, DEVICE_REFRESH },
        { 0b1000000100110011, DEVICE_MEASURE_INIT },
        { 0b0000000100110111, DEVICE_OPERATION_ENABLE },
        { 0b0000000100010111, DEVICE_QUICKSTOP_ACTIVE },
        { 0b0000000100001111, DEVICE_FAULT_REACTION_ACTIVE_DISABLED },
        { 0b0000000100011111, DEVICE_FAULT_REACTION_ACTIVE_ENABLED },
        { 0b0000000100001000, DEVICE_FAULT },
    };
    uint16_t status;
    ESP_RETURN_ON_ERROR(get_statusword(self->node, &status), TAG, "Unable to read status word");
    for (size_t i = 0; i < sizeof(patterns)/sizeof(patterns[0]); ++i) {
        if ((status & mask) == patterns[i].value) {
            *state = patterns[i].state;
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

void 
print_status(OperationMode_t mode, StatusWord_t s)
{
    printf( "ready_switch_on : %d\n"
            "switched_on : %d\n"
            "operation_enable : %d\n"
            "fault : %d\n"
            "voltage_enabled : %d\n"
            "quickstop : %d\n"
            "switch_on_disable : %d\n"
            "warning : %d\n"
            "offset_current_measured : %d\n"
            "remote : %d\n"
            "target_reached : %d\n"
            "internal_limit_active : %d\n"
            "refresh_cycle : %d\n"
            "referenced_to_home : %d\n",
            s.ready_switch_on, s.switched_on, s.operation_enable, s.fault, s.voltage_enabled, s.quickstop,
            s.switch_on_disable, s.warning, s.offset_current_measured, s.remote, s.target_reached, s.internal_limit_active,
            s.refresh_cycle, s.referenced_to_home
         );
    if (mode == PROFILE_POSITION_MODE)
        printf( "setpoint_ack : %d\n"
                "following_error : %d\n",
                s.ppm.setpoint_ack, s.ppm.following_error);
    if (mode == PROFILE_VELOCITY_MODE)
        printf( "speed : %d\n", s.pvm.speed);
    if (mode == HOMING_MODE)
        printf( "homing_attained : %d\n"
                "homing_error : %d\n",
                s.hmm.homing_attained, s.hmm.homing_error);
    if (mode == INTERPOLATED_POSITION_MODE)
        printf( "ipm_active : %d\n", s.ipm.ipm_active);
}

void 
print_controlword(OperationMode_t mode, ControlWord_t c)
{
    printf( "switch_on : %d\n"
            "enable_voltage : %d\n"
            "quickstop : %d\n"
            "enable_operation : %d\n"
            "fault_reset : %d\n"
            "halt : %d\n",
            c.switch_on, c.enable_voltage, c.quickstop,
            c.enable_operation, c.fault_reset, c.halt);
    if (mode == PROFILE_POSITION_MODE)
        printf( "new_setpoint : %d\n"
                "ch_set_immediate : %d\n"
                "relative : %d\n"
                "endless_move : %d\n",
                c.ppm.new_setpoint, c.ppm.ch_set_immediate, c.ppm.relative, c.ppm.endless_move);
    if (mode == HOMING_MODE)
        printf("homing_start : %d\n", c.hmm.homing_start);
    if (mode == INTERPOLATED_POSITION_MODE)
        printf("enable_IPM : %d\n", c.ipm.enable_ipm);
}
