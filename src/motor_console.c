#include "esp_console.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"
#include "epos_motor.h"
#include "epos.h"
#include "esp_check.h"
#include <string.h>

static const char *TAG = "MOTOR";

#define MAX_MOTORS 8
static Motor_t* motors[MAX_MOTORS] = {NULL};

static struct {
    struct arg_str *cmd;
    struct arg_int *node_or_id;
    struct arg_int *param;  // parámetros adicionales según comando
    struct arg_end *end;
} motor_args;

static int find_free_id(void) {
    for(int i = 0; i < MAX_MOTORS; i++) {
        if (motors[i] == NULL) return i;
    }
    return -1;
}

Motor_t* cmd_motor_new(uint8_t node, int* motor_id)
{
    int id = find_free_id();
    if (id < 0) {
        ESP_LOGE(TAG, "No hay slots libres para nuevos motores");
        return NULL;
    }

    Motor_t* motor = motor_new(node);
    if (!motor) {
        ESP_LOGE(TAG, "Error creando motor");
        return NULL;
    }

    motors[id] = motor;
    *motor_id = id;
    return motor;
}

static int cmd_motor(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&motor_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, motor_args.end, argv[0]);
        return 1;
    }

    const char* cmd = motor_args.cmd->sval[0];

    // Comando new es especial porque no necesita ID
    if (strcmp(cmd, "new") == 0) {
        uint8_t node = (uint8_t)motor_args.node_or_id->ival[0];
        int id;
        Motor_t* m = cmd_motor_new(node, &id);
        ESP_LOGI(TAG, "new id %d", id);
        return m != NULL;
    }

    // Para el resto de comandos necesitamos un ID válido
    if (!motor_args.node_or_id->count) {
        ESP_LOGE(TAG, "Falta ID del motor");
        return ESP_FAIL;
    }

    int id = motor_args.node_or_id->ival[0];
    if (id < 0 || id >= MAX_MOTORS || motors[id] == NULL) {
        ESP_LOGE(TAG, "ID de motor %d no válido", id);
        return 1;
    }

    Motor_t* motor = motors[id];

    // Comandos que no requieren parámetros adicionales
    if (strcmp(cmd, "reset") == 0) {
        return motor_reset(motor);
    }
    else if (strcmp(cmd, "cw") == 0) {
        ControlWord_t cw;
        get_controlword(motor->node, &cw.value);
        print_controlword(motor->mode, cw);
        return ESP_OK;
    }
    else if (strcmp(cmd, "status") == 0) {
        static const char* const state_str[] = {
            "START",
            "NOT_READY_TO_SWITCH_ON",
            "SWITCH_ON_DISABLED",
            "READY_TO_SWITCH_ON",
            "SWITCHED_ON",
            "REFRESH",
            "MEASURE_INIT",
            "OPERATION_ENABLE",
            "QUICKSTOP_ACTIVE",
            "FAULT_REACTION_ACTIVE_DISABLED",
            "FAULT_REACTION_ACTIVE_ENABLED",
            "FAULT"
        };

        DeviceState_t state;
        ESP_RETURN_ON_ERROR(motor_get_device_state(motor, &state), TAG, "Unable to read device state");
        ESP_LOGI(TAG, "Motor %d device state: %s (mode=%d)", id,
            (state < sizeof(state_str)/sizeof(state_str[0])) ? state_str[state] : "UNKNOWN", motor->mode);

        StatusWord_t status;
        ESP_RETURN_ON_ERROR(get_statusword(motor->node, &status.value), TAG, "Unable to read status word");
        print_status(motor->mode, status);
        return ESP_OK;
    }
    // Comandos para configuración de perfiles
    else if (strcmp(cmd, "get_profile") == 0) {
        MotorProfile_t profile;
        ESP_RETURN_ON_ERROR(motor_get_profile(motor, &profile), TAG, "Unable to read profile");
        ESP_LOGI(TAG, 
            "Motor %d profile\n velocity=%ld\n acceleration=%ld\n deceleration=%ld\n type=%d\n\n",
            id, profile.velocity, profile.acceleration, profile.deceleration, profile.type);
        return ESP_OK;
    }

    else if (strcmp(cmd, "get_config") == 0) {
        MotorProfileConfig_t config;
        ESP_RETURN_ON_ERROR(motor_get_profile_config(motor, &config), TAG, "Unable to read profile config");
        ESP_LOGI(TAG, 
            "Motor %d profile config:\n position_window=%ld\n window_time=%d\n"
            " min_position=%ld\n max_position=%ld\n"
            " max_velocity=%ld\n max_acceleration=%ld\n"
            " quickstop_deceleration=%ld\n\n",
            id, config.position_window, config.position_window_time,
            config.min_position_limit, config.max_position_limit,
            config.max_velocity, config.max_acceleration, config.quickstop_deceleration);
        return ESP_OK;
    }
    else if (strcmp(cmd, "halt") == 0) {
        motor_halt(motor);
        return ESP_OK;
    }
    else if (strcmp(cmd, "where") == 0) {
        int32_t pos;
        motor_get_actual_position(motor, &pos);
        ESP_LOGI(TAG, "Current position %ld", pos);
        return ESP_OK;
    }

    // Comandos que requieren un parámetro
    if (!motor_args.param->count) {
        ESP_LOGE(TAG, "Falta parámetro para el comando %s", cmd);
        return 1;
    }
    int32_t param = motor_args.param->ival[0];

    if (strcmp(cmd, "abs") == 0) {
        return motor_profile_position_absolute(motor, param);
    }
    else if (strcmp(cmd, "rel") == 0) {
        return motor_profile_position_relative(motor, param);
    }
    else if (strcmp(cmd, "vel") == 0) {
        return motor_profile_velocity_move(motor, param);
    }
    else if (strcmp(cmd, "pos") == 0) {
        return motor_position_set_absolute(motor, param);
    }
    else if (strcmp(cmd, "cur") == 0) {
        return motor_current_set_current(motor, (int16_t)param);
    }
    else if (strcmp(cmd, "v") == 0) {
        return motor_velocity_set_velocity(motor, param);
    }
    else if (strcmp(cmd, "stall") == 0) {
        bool done = false;
        while (!done) {
            ESP_RETURN_ON_ERROR(motor_move_to_stall(motor, param, &done), TAG, "Unable to execute move_to_stall");
        }
        ESP_RETURN_ON_ERROR(motor_current_set_current(motor, 0), TAG, "Unable to halt motor");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Subcomando desconocido: %s", cmd);
    return ESP_FAIL;
}

void motor_register_commands(void)
{
    motor_args.cmd = arg_str1(NULL, NULL, "<cmd>", 
        "Comando (new|reset|abs|rel|vel|cur|pos|v|halt|get_profile|get_config|stall|status|where)");
    motor_args.node_or_id = arg_int0(NULL, NULL, "<node|id>", "Número de nodo (para new) o ID del motor");
    motor_args.param = arg_int0(NULL, NULL, "<pos|delta|velocity|current>", "Parámetro adicional");
    motor_args.end = arg_end(6);

    const esp_console_cmd_t cmd = {
        .command = "motor",
        .help = "Control de motores EPOS\n"
                "  new <node>                 - Crear nuevo motor\n"
                "  reset <id>                 - Resetear motor\n"
                "  abs <id> <pos>             - Mover a posición absoluta PPM\n"
                "  rel <id> <delta>           - Mover posición relativa PPM \n"
                "  vel <id> <velocity>        - Establecer velocidad PVM\n"
                "  pos <id> <pos>             - Establecer posición PM\n"
                "  cur <id> <current>         - Establecer corriente CM\n"
                "  v <id> <velocity>          - Establecer velocidad VM\n"
                "  halt <id>                  - Detener movimiento\n"
                "  get_profile <id>           - Obtener configuración de perfil\n"
                "  get_config <id>            - Obtener configuración\n"
                "  stall <id> <velocity>      - Mover hasta detectar bloqueo\n"
                "  where                      - Posición absoluta actual\n"
                "  status <id>                - Obtener estado del dispositivo",
        .hint = NULL,
        .func = &cmd_motor,
        .argtable = &motor_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
