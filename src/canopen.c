#include "canopen.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define N_ELEMS(x) (sizeof(x)/sizeof((x)[0]))
#define CANOPEN_RX_MSG_QUEUE_LEN 64
#define CANOPEN_TX_TASK_QUEUE_LEN 16
#define CANOPEN_TWAI_TX_QUEUE_DEPTH 32

typedef struct request_s request_t;
typedef struct response_s response_t;

typedef esp_err_t (*run_tx_fn_t) (request_t* self);
typedef void (*run_rx_fn_t) (response_t* self, twai_message_t* msg);

struct request_s {
    run_tx_fn_t run;
    twai_message_t msg;
    uint8_t* value;
    size_t size;
    TaskHandle_t waiter;
    esp_err_t *result_out;
};

typedef bool (*match_rx_fn_t)(const response_t *self, const twai_message_t *msg);

struct response_s {
    run_rx_fn_t run;
    match_rx_fn_t match;
    uint32_t cobid;
    uint8_t* value;
    size_t size;
    TaskHandle_t waiter;
    uint16_t token;
    esp_err_t *result_out;
};

typedef struct {
    bool in_use;
    response_t resp;
} pending_resp_entry_t;

static pending_resp_entry_t pending_resp_table[8];
static portMUX_TYPE pending_resp_mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t cobid_mutex;

#define MAX_CANOPEN_HANDLERS 16

typedef struct canopen_handler_entry {
    bool in_use;
    canopen_handler_fn handler;
    uint32_t cobid;
    void* context;
    struct canopen_handler_entry* next;
} canopen_handler_entry_t;

static canopen_handler_entry_t handler_pool[MAX_CANOPEN_HANDLERS];
static canopen_handler_entry_t* canopen_handlers = NULL;
static SemaphoreHandle_t canopen_handlers_mutex;

static const char* TAG = "CANOPEN";
static TickType_t max_delay = pdMS_TO_TICKS(3000);
static bool enable_dump_msg = false;

static QueueHandle_t rx_msg_queue;
static QueueHandle_t tx_task_queue;
static bool twai_driver_ready = false;
static volatile bool can_bus_off = false;
static volatile bool can_recovering = false;
static volatile bool can_driver_error_event = false;
static volatile bool can_driver_state_change_event = false;
static volatile uint32_t can_driver_error_flags = 0;
static volatile uint32_t can_driver_error_count = 0;
static volatile uint32_t can_driver_state_change_count = 0;
static volatile uint32_t can_driver_rx_queue_drop_count = 0;
static volatile uint32_t can_driver_rx_read_fail_count = 0;
static volatile uint32_t can_driver_async_tx_drop_count = 0;
static portMUX_TYPE can_driver_event_mux = portMUX_INITIALIZER_UNLOCKED;

#define CANOPEN_INTERNAL_STOP_ID   0xFFFFFFFFu

static portMUX_TYPE done_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t done_waiter_task = NULL;
static bool shutdown_requested = false;

static TaskHandle_t dispatch_task_handle;
static TaskHandle_t tx_task_handle;
static TaskHandle_t rx_task_handle;

static void canopen_store_result(request_t *self, esp_err_t err)
{
    if (self != NULL && self->result_out != NULL) {
        *self->result_out = err;
    }
}

static const char *canopen_twai_state_str(twai_state_t state)
{
    switch (state) {
    case TWAI_STATE_STOPPED:
        return "stopped";
    case TWAI_STATE_RUNNING:
        return "running";
    case TWAI_STATE_BUS_OFF:
        return "bus_off";
    case TWAI_STATE_RECOVERING:
        return "recovering";
    default:
        return "unknown";
    }
}

static void canopen_append_flag(char *buf, size_t len, bool *first, const char *text)
{
    size_t used;

    if (buf == NULL || len == 0 || text == NULL) {
        return;
    }

    used = strnlen(buf, len);
    if (used >= len - 1u) {
        return;
    }

    (void)snprintf(buf + used,
                   len - used,
                   "%s%s",
                   *first ? "" : "|",
                   text);
    *first = false;
}

static void canopen_format_error_flags(uint32_t flags, char *buf, size_t len)
{
    bool first = true;

    if (buf == NULL || len == 0) {
        return;
    }

    buf[0] = '\0';
    if (flags == 0u) {
        (void)snprintf(buf, len, "unspecified");
        return;
    }

    if ((flags & (1u << 0)) != 0u) {
        canopen_append_flag(buf, len, &first, "arb_lost");
    }
    if ((flags & (1u << 1)) != 0u) {
        canopen_append_flag(buf, len, &first, "bit");
    }
    if ((flags & (1u << 2)) != 0u) {
        canopen_append_flag(buf, len, &first, "form");
    }
    if ((flags & (1u << 3)) != 0u) {
        canopen_append_flag(buf, len, &first, "stuff");
    }
    if ((flags & (1u << 4)) != 0u) {
        canopen_append_flag(buf, len, &first, "ack");
    }

    if (first) {
        (void)snprintf(buf, len, "0x%08" PRIx32, flags);
    }
}

static void canopen_maybe_log_driver_events(void)
{
    bool log_error = false;
    bool log_state = false;
    uint32_t error_flags = 0;
    uint32_t error_count = 0;
    uint32_t rx_queue_drop_count = 0;
    uint32_t rx_read_fail_count = 0;
    uint32_t async_tx_drop_count = 0;
    twai_status_info_t status = {0};
    bool have_info = false;
    uint32_t alerts = 0;

    if (twai_driver_ready && twai_read_alerts(&alerts, 0) == ESP_OK && alerts != 0u) {
        taskENTER_CRITICAL(&can_driver_event_mux);
        if ((alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ARB_LOST |
                       TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS |
                       TWAI_ALERT_ABOVE_ERR_WARN)) != 0u) {
            can_driver_error_event = true;
            can_driver_error_flags |= alerts;
            can_driver_error_count++;
        }
        if ((alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED)) != 0u) {
            can_driver_state_change_event = true;
            can_driver_state_change_count++;
            if ((alerts & TWAI_ALERT_BUS_OFF) != 0u) {
                can_bus_off = true;
            }
            if ((alerts & TWAI_ALERT_BUS_RECOVERED) != 0u) {
                can_bus_off = false;
                can_recovering = false;
            }
        }
        if ((alerts & TWAI_ALERT_RX_QUEUE_FULL) != 0u) {
            can_driver_rx_queue_drop_count++;
        }
        taskEXIT_CRITICAL(&can_driver_event_mux);
    }

    taskENTER_CRITICAL(&can_driver_event_mux);
    if (can_driver_error_event) {
        log_error = true;
        can_driver_error_event = false;
        error_flags = can_driver_error_flags;
        can_driver_error_flags = 0;
        error_count = can_driver_error_count;
        can_driver_error_count = 0;
    }
    if (can_driver_state_change_event) {
        log_state = true;
        can_driver_state_change_count = 0;
        can_driver_state_change_event = false;
    }
    rx_queue_drop_count = can_driver_rx_queue_drop_count;
    can_driver_rx_queue_drop_count = 0;
    rx_read_fail_count = can_driver_rx_read_fail_count;
    can_driver_rx_read_fail_count = 0;
    async_tx_drop_count = can_driver_async_tx_drop_count;
    can_driver_async_tx_drop_count = 0;
    taskEXIT_CRITICAL(&can_driver_event_mux);

    if ((log_error || log_state || rx_queue_drop_count > 0u || async_tx_drop_count > 0u) &&
        twai_driver_ready &&
        twai_get_status_info(&status) == ESP_OK) {
        have_info = true;
    }

    if (log_error) {
        char flags_buf[48];
        canopen_format_error_flags(error_flags, flags_buf, sizeof(flags_buf));
        if (have_info) {
            ESP_LOGW(TAG,
                     "TWAI error event: flags=%s tx_err=%u rx_err=%u bus_err=%" PRIu32 " state=%s%s",
                     flags_buf,
                     (unsigned) status.tx_error_counter,
                     (unsigned) status.rx_error_counter,
                     status.bus_error_count,
                     canopen_twai_state_str(status.state),
                     error_count > 1u ? " (coalesced)" : "");
        } else {
            ESP_LOGW(TAG,
                     "TWAI error event: flags=%s%s",
                     flags_buf,
                     error_count > 1u ? " (coalesced)" : "");
        }
    }

    if (log_state) {
        if (have_info) {
            ESP_LOGW(TAG,
                     "TWAI state change: state=%s tx_err=%u rx_err=%u bus_err=%" PRIu32,
                     canopen_twai_state_str(status.state),
                     (unsigned) status.tx_error_counter,
                     (unsigned) status.rx_error_counter,
                     status.bus_error_count);
        } else {
            ESP_LOGW(TAG, "TWAI state change");
        }
    }

    if (rx_read_fail_count > 0u) {
        ESP_LOGW(TAG,
                 "TWAI RX read failures: dropped=%" PRIu32,
                 rx_read_fail_count);
    }

    if (rx_queue_drop_count > 0u) {
        if (have_info) {
            ESP_LOGW(TAG,
                     "TWAI RX queue overflow: dropped=%" PRIu32 " queue_len=%u tx_err=%u rx_err=%u bus_err=%" PRIu32 " state=%s",
                     rx_queue_drop_count,
                     (unsigned) CANOPEN_RX_MSG_QUEUE_LEN,
                     (unsigned) status.tx_error_counter,
                     (unsigned) status.rx_error_counter,
                     status.bus_error_count,
                     canopen_twai_state_str(status.state));
        } else {
            ESP_LOGW(TAG,
                     "TWAI RX queue overflow: dropped=%" PRIu32 " queue_len=%u",
                     rx_queue_drop_count,
                     (unsigned) CANOPEN_RX_MSG_QUEUE_LEN);
        }
    }

    if (async_tx_drop_count > 0u) {
        if (have_info) {
            uint32_t tx_remaining = status.msgs_to_tx < CANOPEN_TWAI_TX_QUEUE_DEPTH
                                  ? (CANOPEN_TWAI_TX_QUEUE_DEPTH - status.msgs_to_tx)
                                  : 0u;
            ESP_LOGW(TAG,
                     "TWAI async TX saturated: dropped=%" PRIu32 " tx_queue_remaining=%" PRIu32 " tx_err=%u rx_err=%u bus_err=%" PRIu32 " state=%s",
                     async_tx_drop_count,
                     tx_remaining,
                     (unsigned) status.tx_error_counter,
                     (unsigned) status.rx_error_counter,
                     status.bus_error_count,
                     canopen_twai_state_str(status.state));
        } else {
            ESP_LOGW(TAG,
                     "TWAI async TX saturated: dropped=%" PRIu32,
                     async_tx_drop_count);
        }
    }
}

static esp_err_t canopen_try_recover_bus(void)
{
    twai_status_info_t status = {0};
    esp_err_t err;

    if (!twai_driver_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    err = twai_get_status_info(&status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_get_status_info failed: %s", esp_err_to_name(err));
        return err;
    }

    if (status.state == TWAI_STATE_STOPPED) {
        err = twai_start();
        if (err == ESP_OK) {
            can_bus_off = false;
            can_recovering = false;
        }
        return err;
    }

    if (status.state != TWAI_STATE_BUS_OFF && status.state != TWAI_STATE_RECOVERING) {
        can_bus_off = false;
        return ESP_OK;
    }

    if (can_recovering) {
        for (int i = 0; i < 200; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
            err = twai_get_status_info(&status);
            if (err == ESP_OK && status.state == TWAI_STATE_STOPPED) {
                err = twai_start();
                if (err == ESP_OK) {
                    status.state = TWAI_STATE_RUNNING;
                }
            }
            if (err == ESP_OK && status.state == TWAI_STATE_RUNNING) {
                can_bus_off = false;
                can_recovering = false;
                ESP_LOGW(TAG, "TWAI bus recovered");
                return ESP_OK;
            }
        }
        return ESP_ERR_TIMEOUT;
    }

    can_recovering = true;
    ESP_LOGW(TAG, "TWAI bus-off detected, starting recovery");
    err = twai_initiate_recovery();
    if (err != ESP_OK) {
        can_recovering = false;
        ESP_LOGE(TAG, "twai_initiate_recovery failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < 200; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
        err = twai_get_status_info(&status);
        if (err == ESP_OK && status.state == TWAI_STATE_STOPPED) {
            err = twai_start();
            if (err == ESP_OK) {
                status.state = TWAI_STATE_RUNNING;
            }
        }
        if (err == ESP_OK && status.state == TWAI_STATE_RUNNING) {
            can_bus_off = false;
            can_recovering = false;
            ESP_LOGW(TAG, "TWAI bus recovered");
            return ESP_OK;
        }
    }

    can_recovering = false;
    ESP_LOGE(TAG, "TWAI recovery timeout");
    return ESP_ERR_TIMEOUT;
}

static bool match_cobid_only(const response_t *self, const twai_message_t *msg)
{
    return msg->identifier == self->cobid;
}

static esp_err_t canopen_add_pending_response(const response_t *resp)
{
    esp_err_t err = ESP_ERR_NO_MEM;

    taskENTER_CRITICAL(&pending_resp_mux);
    for (int i = 0; i < N_ELEMS(pending_resp_table); ++i) {
        if (!pending_resp_table[i].in_use) {
            pending_resp_table[i].resp = *resp;
            pending_resp_table[i].in_use = true;
            err = ESP_OK;
            break;
        }
    }
    taskEXIT_CRITICAL(&pending_resp_mux);

    return err;
}

static bool canopen_take_pending_response(const twai_message_t *msg, response_t *out)
{
    bool found = false;

    taskENTER_CRITICAL(&pending_resp_mux);
    for (int i = 0; i < N_ELEMS(pending_resp_table); ++i) {
        if (pending_resp_table[i].in_use &&
            pending_resp_table[i].resp.match &&
            pending_resp_table[i].resp.match(&pending_resp_table[i].resp, msg)) {
            *out = pending_resp_table[i].resp;
            pending_resp_table[i].in_use = false;
            found = true;
            break;
        }
    }
    taskEXIT_CRITICAL(&pending_resp_mux);

    return found;
}

static void dump_msg(const char* info, twai_message_t* msg)
{
    if (enable_dump_msg) {
        uint8_t* d = msg->data;
        ESP_LOGI(TAG, "%s %08x (%d bytes) %02x %02x %02x %02x %02x %02x %02x %02x",
                 info, (unsigned)msg->identifier, (int)msg->data_length_code,
                 d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    }
}

/*  Asynchronous CANopen handlers */

#define MAX_MATCHING 8

typedef struct {
    canopen_handler_fn handler;
    uint32_t cobid;
    void *context;
} handler_call_t;

static void canopen_receive_canopen(twai_message_t* msg)
{
    handler_call_t matches[MAX_MATCHING];
    int count = 0;

    xSemaphoreTake(canopen_handlers_mutex, portMAX_DELAY);
    canopen_handler_entry_t* current = canopen_handlers;
    while (current != NULL && count < MAX_MATCHING) {
        if (current->cobid == msg->identifier) {
            matches[count].handler = current->handler;
            matches[count].cobid = current->cobid;
            matches[count].context = current->context;
            count++;
        }
        current = current->next;
    }
    xSemaphoreGive(canopen_handlers_mutex);

    for (int i = 0; i < count; ++i) {
        if (matches[i].handler == NULL) {
            ESP_LOGE(TAG, "NULL CANopen handler for COB-ID %08x",
                    (unsigned)matches[i].cobid);
            continue;
        }
        matches[i].handler(matches[i].cobid, msg->data, matches[i].context);
    }
}

static esp_err_t canopen_register_handler_nolock(uint32_t cobid,
                                                  canopen_handler_fn handler_fn,
                                                  void *context,
                                                  canopen_handler_handle_t *out_handle)
{
    canopen_handler_entry_t *entry = NULL;

    if (handler_fn == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < MAX_CANOPEN_HANDLERS; ++i) {
        if (!handler_pool[i].in_use) {
            entry = &handler_pool[i];
            entry->in_use = true;
            break;
        }
    }

    if (entry == NULL) {
        return ESP_ERR_NO_MEM;
    }

    entry->cobid = cobid;
    entry->handler = handler_fn;
    entry->context = context;
    entry->next = canopen_handlers;
    canopen_handlers = entry;

    *out_handle = entry;
    return ESP_OK;
}

static esp_err_t canopen_unregister_handler_nolock(canopen_handler_handle_t handle)
{
    canopen_handler_entry_t *current = canopen_handlers;
    canopen_handler_entry_t *prev = NULL;

    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    while (current != NULL) {
        if (current == handle) {
            if (prev == NULL) {
                canopen_handlers = current->next;
            } else {
                prev->next = current->next;
            }

            current->in_use = false;
            current->cobid = 0;
            current->handler = NULL;
            current->context = NULL;
            current->next = NULL;

            return ESP_OK;
        }

        prev = current;
        current = current->next;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t canopen_register_handler(uint32_t cobid,
                                   canopen_handler_fn handler_fn,
                                   void *context,
                                   canopen_handler_handle_t *out_handle)
{
    esp_err_t err;

    ESP_RETURN_ON_FALSE(handler_fn != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Invalid CANopen handler for %08x", (unsigned)cobid);
    ESP_RETURN_ON_FALSE(out_handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "NULL out_handle");

    xSemaphoreTake(canopen_handlers_mutex, portMAX_DELAY);
    err = canopen_register_handler_nolock(cobid, handler_fn, context, out_handle);
    xSemaphoreGive(canopen_handlers_mutex);

    if (err == ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "No free handler slots");
    }

    return err;
}

esp_err_t canopen_unregister_handler(canopen_handler_handle_t handle)
{
    esp_err_t err;

    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "NULL handler handle");

    xSemaphoreTake(canopen_handlers_mutex, portMAX_DELAY);
    err = canopen_unregister_handler_nolock(handle);
    xSemaphoreGive(canopen_handlers_mutex);

    return err;
}

static esp_err_t canopen_send_now(request_t* self)
{
    esp_err_t err;

    if (!twai_driver_ready) {
        ESP_LOGE(TAG, "TWAI driver not initialized");
        canopen_store_result(self, ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    if (can_bus_off) {
        err = canopen_try_recover_bus();
        if (err != ESP_OK) {
            canopen_store_result(self, err);
            return err;
        }
    }

    dump_msg("Transmit", &self->msg);
    err = twai_transmit(&self->msg, max_delay);
    if (err == ESP_ERR_INVALID_STATE || can_bus_off) {
        esp_err_t rec_err = canopen_try_recover_bus();
        if (rec_err == ESP_OK) {
            err = twai_transmit(&self->msg, max_delay);
        } else if (err == ESP_OK) {
            err = rec_err;
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_transmit failed: %s", esp_err_to_name(err));
        canopen_store_result(self, err);
        return err;
    }

    canopen_store_result(self, ESP_OK);
    return ESP_OK;
}

static esp_err_t canopen_post_now(request_t *self)
{
    esp_err_t err;
    twai_status_info_t status = {0};

    if (!twai_driver_ready) {
        ESP_LOGE(TAG, "TWAI driver not initialized");
        canopen_store_result(self, ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    if (can_bus_off) {
        err = canopen_try_recover_bus();
        if (err != ESP_OK) {
            canopen_store_result(self, err);
            return err;
        }
    }

    err = twai_get_status_info(&status);
    if (err == ESP_OK && status.msgs_to_tx >= CANOPEN_TWAI_TX_QUEUE_DEPTH) {
        taskENTER_CRITICAL(&can_driver_event_mux);
        can_driver_async_tx_drop_count++;
        taskEXIT_CRITICAL(&can_driver_event_mux);
        canopen_store_result(self, ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    dump_msg("Post", &self->msg);
    err = twai_transmit(&self->msg, 0);
    if (err == ESP_ERR_INVALID_STATE || can_bus_off) {
        esp_err_t rec_err = canopen_try_recover_bus();
        if (rec_err == ESP_OK) {
            err = twai_transmit(&self->msg, 0);
        } else if (err == ESP_OK) {
            err = rec_err;
        }
    }
    if (err != ESP_OK) {
        if (err == ESP_ERR_TIMEOUT) {
            taskENTER_CRITICAL(&can_driver_event_mux);
            can_driver_async_tx_drop_count++;
            taskEXIT_CRITICAL(&can_driver_event_mux);
        }
        canopen_store_result(self, err);
        return err;
    }

    canopen_store_result(self, ESP_OK);
    return ESP_OK;
}

static esp_err_t canopen_send_notify(request_t *self)
{
    esp_err_t err = canopen_send_now(self);
    if (self->waiter != NULL) {
        xTaskNotifyGive(self->waiter);
    }
    return err;
}

esp_err_t canopen_post(const twai_message_t *msg)
{
    request_t req;

    ESP_RETURN_ON_FALSE(msg != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL msg");
    ESP_RETURN_ON_FALSE(tx_task_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "CANopen not initialized");

    req = (request_t){
        .run = canopen_post_now,
        .msg = *msg,
        .value = NULL,
        .size = 0,
        .waiter = NULL,
        .result_out = NULL,
    };

    if (xQueueSend(tx_task_queue, &req, 0) == pdTRUE) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "CANopen TX queue full, dropping async frame id=0x%03" PRIx32, msg->identifier);
    return ESP_ERR_TIMEOUT;
}

esp_err_t canopen_send(const twai_message_t *msg)
{
    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    request_t req;
    esp_err_t result = ESP_OK;

    ESP_RETURN_ON_FALSE(msg != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL msg");
    ESP_RETURN_ON_FALSE(tx_task_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "CANopen not initialized");

    (void) ulTaskNotifyTake(pdTRUE, 0);

    req = (request_t){
        .run = canopen_send_notify,
        .msg = *msg,
        .value = NULL,
        .size = 0,
        .waiter = waiter,
        .result_out = &result,
    };

    if (xQueueSend(tx_task_queue, &req, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (ulTaskNotifyTake(pdTRUE, max_delay) != pdTRUE) {
        ESP_LOGI(TAG, "Sender thread is blocked, unable to transmit CAN frame");
        return ESP_FAIL;
    }

    return result;
}

static void sdo_download_response(response_t* self, twai_message_t* msg)
{
    dump_msg("Receive", msg);
    SDO_download_resp_t* resp = (SDO_download_resp_t*) msg->data;
    if (resp->scs != SDO_SCS_DOWNLOAD) {
        dump_msg("Wrong response code", msg);
    }
    xTaskNotifyGive(self->waiter);
}

static void sdo_download_segment_response(response_t* self, twai_message_t* msg);

static esp_err_t sdo_download_segment_request(request_t* self)
{
    response_t resp = { 
        .run = sdo_download_segment_response, 
        .cobid = self->msg.identifier - 0x600 + 0x580, 
        .value = self->value, 
        .size = self->size, 
        .waiter = self->waiter,
        .result_out = self->result_out
    };
    resp.match = match_cobid_only;
    esp_err_t err = canopen_add_pending_response(&resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No free pending response slots");
        canopen_store_result(self, err);
        if (self->waiter != NULL) {
            xTaskNotifyGive(self->waiter);
        }
        return err;
    }
    return canopen_send_now(self);
}

static void sdo_download_segment_response(response_t* self, twai_message_t* msg)
{
    dump_msg("Rx Seg", msg);
    twai_message_t req_msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };
    if (self->size > 0) {
        SDO_download_seg_resp_t* resp = (SDO_download_seg_resp_t*) msg->data;
        SDO_download_seg_req_t* req_payload = (SDO_download_seg_req_t*) req_msg.data;
        size_t n = (self->size < 7 ? self->size : 7);
        *req_payload = (SDO_download_seg_req_t){ .ccs = SDO_CCS_DOWNLOAD_SEG,
                                                 .t = (resp->scs != SDO_SCS_DOWNLOAD && !resp->t),
                                                 .c = (n == self->size) };
        memcpy(req_payload->seg_data, self->value, n);
        request_t req = { .run = sdo_download_segment_request, .msg = req_msg, .value = self->value + n, .size = self->size - n, .waiter = self->waiter, .result_out = self->result_out };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    } else {
        xTaskNotifyGive(self->waiter);
    }
}

static esp_err_t sdo_download_request(request_t* self)
{
    SDO_download_req_t* payload = (SDO_download_req_t*) self->msg.data;
    if (payload->e) {
        response_t resp = { 
            .run =  sdo_download_response, 
            .cobid = self->msg.identifier - 0x600 + 0x580, 
            .waiter = self->waiter,
            .result_out = self->result_out
        };
        resp.match = match_cobid_only;
        esp_err_t err = canopen_add_pending_response(&resp);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "No free pending response slots");
            canopen_store_result(self, err);
            if (self->waiter != NULL) {
                xTaskNotifyGive(self->waiter);
            }
            return err;
        }
    } else {
        response_t resp = { 
            .run =  sdo_download_segment_response, 
            .cobid = self->msg.identifier - 0x600 + 0x580, 
            .value = self->value, 
            .size = self->size, 
            .waiter = self->waiter,
            .result_out = self->result_out
        };
        resp.match = match_cobid_only;
        esp_err_t err = canopen_add_pending_response(&resp);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "No free pending response slots");
            canopen_store_result(self, err);
            if (self->waiter != NULL) {
                xTaskNotifyGive(self->waiter);
            }
            return err;
        }
    }
    return canopen_send_now(self);
}

esp_err_t sdo_download(uint32_t id, uint16_t index, uint8_t subindex, void* value, size_t size)
{
    esp_err_t result = ESP_OK;
    twai_message_t msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = id, .data_length_code = 8 };
    SDO_download_req_t* payload = (SDO_download_req_t*) msg.data;
    if (size <= 4) {
        *payload = (SDO_download_req_t){ .s = 1, .e = 1, .n = 4 - size, .x = 0, .ccs = SDO_CCS_DOWNLOAD, .index = index, .subindex = subindex};
        memcpy(payload->d, value, size);
    } else {
        *payload = (SDO_download_req_t){ .s = 1, .e = 0, .n = 0, .x = 0, .ccs = SDO_CCS_DOWNLOAD, .index = index, .subindex = subindex, .dsize = size };
    }
    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0);
    request_t req = { .run = sdo_download_request, .msg = msg, .value = value, .size = size, .waiter = waiter, .result_out = &result };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (ulTaskNotifyTake(pdTRUE, max_delay) != pdTRUE) {
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        return ESP_FAIL;
    }
    return result;
}

static esp_err_t sdo_upload_segment_request(request_t* self);

static void sdo_upload_segment_response(response_t* self, twai_message_t* msg)
{
    dump_msg("Rx Seg", msg);
    SDO_upload_seq_resp_t* payload = (SDO_upload_seq_resp_t*) msg->data;
    size_t n = 7 - payload->n;
    memcpy(self->value, payload->seg_data, n);
    if (payload->c) {
        xTaskNotifyGive(self->waiter);
    } else {
        twai_message_t req_msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };
        SDO_upload_seq_req_t* req_payload = (SDO_upload_seq_req_t*) req_msg.data;
        *req_payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = !payload->t, .reserved = {0} };
        request_t req = { .run = sdo_upload_segment_request, .msg = req_msg, .value = self->value + n, .waiter = self->waiter, .result_out = self->result_out };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
}

static esp_err_t sdo_upload_segment_request(request_t* self)
{
    response_t resp = { 
        .run = sdo_upload_segment_response, 
        .cobid = self->msg.identifier - 0x600 + 0x580, 
        .value = self->value, 
        .waiter = self->waiter,
        .result_out = self->result_out
    };
    resp.match = match_cobid_only;
    esp_err_t err = canopen_add_pending_response(&resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No free pending response slots");
        canopen_store_result(self, err);
        if (self->waiter != NULL) {
            xTaskNotifyGive(self->waiter);
        }
        return err;
    }
    return canopen_send_now(self);
}

static void sdo_upload_response(response_t* self, twai_message_t* msg)
{
    dump_msg("Receive", msg);
    SDO_upload_resp_t* payload = (SDO_upload_resp_t*) msg->data;
    size_t n = 4 - payload->n;
    if (n > 0)
        memcpy(self->value, payload->d, 4 - payload->n);
    if (payload->e) {
        xTaskNotifyGive(self->waiter);
    } else {
        twai_message_t req_msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };
        SDO_upload_seq_req_t* payload = (SDO_upload_seq_req_t*) req_msg.data;
        *payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = 0, .reserved = {0} };
        request_t req = { .run = sdo_upload_segment_request, .msg = req_msg, .value = self->value + n, .waiter = self->waiter, .result_out = self->result_out };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
}

static esp_err_t sdo_upload_request(request_t *self)
{
    response_t resp = {
        .run = sdo_upload_response,
        .match = match_cobid_only,
        .cobid = self->msg.identifier - 0x600 + 0x580,
        .value = self->value,
        .waiter = self->waiter,
        .result_out = self->result_out
    };

    esp_err_t err = canopen_add_pending_response(&resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No free pending response slots");
        canopen_store_result(self, err);
        if (self->waiter != NULL) {
            xTaskNotifyGive(self->waiter);
        }
        return err;
    }

    return canopen_send_now(self);
}

esp_err_t sdo_upload(uint32_t id, uint16_t index, uint8_t subindex, void *ret)
{
    esp_err_t result = ESP_OK;

    twai_message_t msg = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = id,
        .data_length_code = 8
    };

    SDO_upload_req_t *payload = (SDO_upload_req_t *) msg.data;
    *payload = (SDO_upload_req_t) {
        .x = 0,
        .ccs = SDO_CCS_UPLOAD,
        .index = index,
        .subindex = subindex,
        .reserved = {0}
    };

    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0);

    request_t req = {
        .run = sdo_upload_request,
        .msg = msg,
        .value = ret,
        .size = 0,
        .waiter = waiter,
        .result_out = &result
    };

    if (xQueueSend(tx_task_queue, &req, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (ulTaskNotifyTake(pdTRUE, max_delay) != pdTRUE) {
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        return ESP_FAIL;
    }

    return result;
}

esp_err_t nmt(uint8_t cs, uint8_t n)
{
    twai_message_t msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = 0, .data_length_code = 2, .data = { cs, n, 0, 0 } };
    return canopen_send(&msg);
}

void canopen_request_shutdown(void)
{
    taskENTER_CRITICAL(&done_mux);
    shutdown_requested = true;
    TaskHandle_t waiter = done_waiter_task;
    taskEXIT_CRITICAL(&done_mux);

    if (waiter != NULL) {
        xTaskNotifyGive(waiter);
    }
}

static twai_timing_config_t canopen_make_timing_config(uint32_t bitrate)
{
    switch (bitrate) {
    case 25000u:   return (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
    case 50000u:   return (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
    case 100000u:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
    case 125000u:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
    case 250000u:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
    case 500000u:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
#ifdef TWAI_TIMING_CONFIG_800KBITS
    case 800000u:  return (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
#endif
    case 1000000u: return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    default:
        ESP_LOGW(TAG, "Unsupported TWAI bitrate %" PRIu32 ", using 1 Mbit/s", bitrate);
        return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    }
}

static void canopen_receive_task(void *arg)
{
    (void)arg;

    for (;;) {
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, portMAX_DELAY);

        bool stop;
        taskENTER_CRITICAL(&done_mux);
        stop = shutdown_requested;
        taskEXIT_CRITICAL(&done_mux);

        if (stop) {
            break;
        }

        if (err != ESP_OK) {
            taskENTER_CRITICAL(&can_driver_event_mux);
            can_driver_rx_read_fail_count++;
            taskEXIT_CRITICAL(&can_driver_event_mux);
            canopen_maybe_log_driver_events();
            continue;
        }

        if (xQueueSend(rx_msg_queue, &msg, 0) != pdTRUE) {
            taskENTER_CRITICAL(&can_driver_event_mux);
            can_driver_rx_queue_drop_count++;
            taskEXIT_CRITICAL(&can_driver_event_mux);
        }
    }
    vTaskDelete(NULL);
}

static void canopen_transmit_task(void *arg)
{
    for (;;) {
        canopen_maybe_log_driver_events();
        request_t req;
        if (xQueueReceive(tx_task_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (req.run == NULL) {
            ESP_LOGW(TAG, "Stopping CANopen transmit task");
            break;
        }
        (void) req.run(&req);
    }
    vTaskDelete(NULL);
}

static void canopen_dispatch_task(void *arg)
{
    twai_message_t msg;
    response_t resp;

    for (;;) {
        canopen_maybe_log_driver_events();
        if (xQueueReceive(rx_msg_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (msg.identifier == CANOPEN_INTERNAL_STOP_ID) break;
        if (canopen_take_pending_response(&msg, &resp)) {
            if (resp.run == NULL) {
                ESP_LOGE(TAG, "NULL pending response callback for COB-ID %08x",
                        (unsigned)resp.cobid);
                continue;
            }
            resp.run(&resp, &msg);
            continue;
        }
        canopen_receive_canopen(&msg);
    }
    vTaskDelete(NULL);
}

esp_err_t canopen_initialize(const canopen_init_cfg_t *cfg)
{
    static canopen_init_cfg_t default_cfg = CANOPEN_INIT_DEFAULT();
    uint32_t bitrate;
    twai_timing_config_t t_config;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_general_config_t g_config;

    if (cfg == NULL)
        cfg = &default_cfg;

    max_delay = pdMS_TO_TICKS(cfg->max_delay_ms);
    enable_dump_msg = cfg->enable_dump_msg;
    bitrate = cfg->can_bitrate != 0u ? cfg->can_bitrate : DEFAULT_CAN_BITRATE;
    t_config = canopen_make_timing_config(bitrate);
    g_config = (twai_general_config_t)TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)cfg->can_tx_pin,
                                                                  (gpio_num_t)cfg->can_rx_pin,
                                                                  TWAI_MODE_NORMAL);
    g_config.tx_queue_len = CANOPEN_TWAI_TX_QUEUE_DEPTH;
    g_config.rx_queue_len = CANOPEN_RX_MSG_QUEUE_LEN;
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED |
                              TWAI_ALERT_BUS_ERROR |
                              TWAI_ALERT_ARB_LOST |
                              TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_ABOVE_ERR_WARN |
                              TWAI_ALERT_RX_QUEUE_FULL;

    rx_msg_queue = xQueueCreate(CANOPEN_RX_MSG_QUEUE_LEN, sizeof(twai_message_t));
    tx_task_queue = xQueueCreate(CANOPEN_TX_TASK_QUEUE_LEN, sizeof(request_t));
    canopen_handlers_mutex = xSemaphoreCreateMutex();
    cobid_mutex = xSemaphoreCreateMutex();

    ESP_RETURN_ON_FALSE(rx_msg_queue != NULL && tx_task_queue != NULL &&
                        canopen_handlers_mutex != NULL && cobid_mutex != NULL,
                        ESP_ERR_NO_MEM, TAG, "Unable to allocate CANopen resources");

    ESP_RETURN_ON_ERROR(twai_driver_install(&g_config, &t_config, &f_config), TAG, "Unable to install TWAI driver");
    twai_driver_ready = true;
    ESP_RETURN_ON_ERROR(twai_start(), TAG, "Unable to start TWAI driver");

    taskENTER_CRITICAL(&done_mux);
    shutdown_requested = false;
    done_waiter_task = NULL;
    taskEXIT_CRITICAL(&done_mux);

    can_bus_off = false;
    can_recovering = false;
    taskENTER_CRITICAL(&can_driver_event_mux);
    can_driver_error_event = false;
    can_driver_state_change_event = false;
    can_driver_error_flags = 0;
    can_driver_error_count = 0;
    can_driver_state_change_count = 0;
    can_driver_rx_queue_drop_count = 0;
    can_driver_rx_read_fail_count = 0;
    can_driver_async_tx_drop_count = 0;
    taskEXIT_CRITICAL(&can_driver_event_mux);

    ESP_LOGI(TAG,
             "TWAI init: tx=%d rx=%d bitrate=%" PRIu32 " timeout_ms=%" PRIu32 " txq=%u",
             cfg->can_tx_pin,
             cfg->can_rx_pin,
             bitrate,
             (uint32_t) cfg->max_delay_ms,
             (unsigned) CANOPEN_TWAI_TX_QUEUE_DEPTH);

    xTaskCreatePinnedToCore(canopen_receive_task, "receive", 4096, NULL, 8, &rx_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(canopen_dispatch_task, "dispatch", 4096, NULL, 8, &dispatch_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(canopen_transmit_task, "transmit", 4096, NULL, 9, &tx_task_handle, tskNO_AFFINITY);

    return ESP_OK;
}

TickType_t canopen_get_max_delay_ms(void)
{
    return pdTICKS_TO_MS(max_delay);
}

void canopen_max_delay_ms(unsigned delay)
{
    max_delay = pdMS_TO_TICKS(delay);
}

bool canopen_is_dump_enabled(void)
{
    return enable_dump_msg;
}

void canopen_dump_enabled(bool enable)
{
    enable_dump_msg = enable;
}

esp_err_t canopen_wait_shutdown(void)
{
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0);

    taskENTER_CRITICAL(&done_mux);
    done_waiter_task = self;
    bool already_requested = shutdown_requested;
    taskEXIT_CRITICAL(&done_mux);

    if (!already_requested) {
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    if (twai_driver_ready) {
        ESP_RETURN_ON_ERROR(twai_stop(), TAG, "Unable to stop TWAI driver");
    }

    twai_message_t stop_msg = {
        .identifier = CANOPEN_INTERNAL_STOP_ID,
        .data_length_code = 0,
        .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0
    };
    (void) xQueueSend(rx_msg_queue, &stop_msg, 0);

    request_t stop_req = { 0 };
    stop_req.run = NULL;
    (void) xQueueSend(tx_task_queue, &stop_req, 0);

    vTaskDelay(pdMS_TO_TICKS(20));

    if (twai_driver_ready) {
        ESP_RETURN_ON_ERROR(twai_driver_uninstall(), TAG, "Unable to uninstall TWAI driver");
        twai_driver_ready = false;
    }

    if (rx_msg_queue) {
        vQueueDelete(rx_msg_queue);
        rx_msg_queue = NULL;
    }

    if (tx_task_queue) {
        vQueueDelete(tx_task_queue);
        tx_task_queue = NULL;
    }

    if (canopen_handlers_mutex) {
        vSemaphoreDelete(canopen_handlers_mutex);
        canopen_handlers_mutex = NULL;
    }

    if (cobid_mutex) {
        vSemaphoreDelete(cobid_mutex);
        cobid_mutex = NULL;
    }

    taskENTER_CRITICAL(&done_mux);
    shutdown_requested = false;
    done_waiter_task = NULL;
    rx_task_handle = NULL;
    dispatch_task_handle = NULL;
    tx_task_handle = NULL;
    taskEXIT_CRITICAL(&done_mux);

    can_bus_off = false;
    can_recovering = false;
    taskENTER_CRITICAL(&can_driver_event_mux);
    can_driver_error_event = false;
    can_driver_state_change_event = false;
    can_driver_error_count = 0;
    can_driver_state_change_count = 0;
    taskEXIT_CRITICAL(&can_driver_event_mux);

    return ESP_OK;
}

typedef struct {
    uint32_t cobid;
    TaskHandle_t waiter;
    void* ret;
    bool in_use;
} cobid_entry_t;

static cobid_entry_t cobid_table[] = {
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
    { .in_use = false },
};

static void canopen_signal_cobid(uint32_t cobid, void* data, void* context)
{
    TaskHandle_t waiter = NULL;
    void* ret = NULL;

    (void) context;

    xSemaphoreTake(cobid_mutex, portMAX_DELAY);
    for (int i = 0; i < N_ELEMS(cobid_table); i++) {
        if (cobid_table[i].in_use && cobid_table[i].cobid == cobid) {
            waiter = cobid_table[i].waiter;
            ret = cobid_table[i].ret;

            cobid_table[i].in_use = false;
            cobid_table[i].waiter = NULL;
            cobid_table[i].ret = NULL;
            break;
        }
    }
    xSemaphoreGive(cobid_mutex);

    if (waiter != NULL) {
        if (ret != NULL) {
            memcpy(ret, data, 8);
        }
        xTaskNotifyGive(waiter);
    }
}

esp_err_t canopen_wait_until(uint32_t cobid, void* ret)
{
    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    canopen_handler_handle_t handler_handle = NULL;
    int slot = -1;
    esp_err_t err;
    BaseType_t done;

    (void) ulTaskNotifyTake(pdTRUE, 0);

    xSemaphoreTake(canopen_handlers_mutex, portMAX_DELAY);
    xSemaphoreTake(cobid_mutex, portMAX_DELAY);

    for (int i = 0; i < N_ELEMS(cobid_table); ++i) {
        if (!cobid_table[i].in_use) {
            cobid_table[i].cobid = cobid;
            cobid_table[i].waiter = waiter;
            cobid_table[i].ret = ret;
            cobid_table[i].in_use = true;
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        xSemaphoreGive(cobid_mutex);
        xSemaphoreGive(canopen_handlers_mutex);
        ESP_LOGE(TAG, "No free entries in COB-ID table for %08x", (unsigned)cobid);
        return ESP_ERR_NO_MEM;
    }

    err = canopen_register_handler_nolock(cobid, canopen_signal_cobid, NULL, &handler_handle);
    if (err != ESP_OK) {
        cobid_table[slot].in_use = false;
        cobid_table[slot].waiter = NULL;
        cobid_table[slot].ret = NULL;
        xSemaphoreGive(cobid_mutex);
        xSemaphoreGive(canopen_handlers_mutex);

        ESP_LOGE(TAG, "Unable to register handler for COB-ID %08x", (unsigned)cobid);
        return err;
    }

    xSemaphoreGive(cobid_mutex);
    xSemaphoreGive(canopen_handlers_mutex);

    done = ulTaskNotifyTake(pdTRUE, max_delay);

    xSemaphoreTake(canopen_handlers_mutex, portMAX_DELAY);
    xSemaphoreTake(cobid_mutex, portMAX_DELAY);

    if (handler_handle != NULL) {
        (void) canopen_unregister_handler_nolock(handler_handle);
    }

    if (!done) {
        cobid_table[slot].in_use = false;
        cobid_table[slot].waiter = NULL;
        cobid_table[slot].ret = NULL;
    }

    xSemaphoreGive(cobid_mutex);
    xSemaphoreGive(canopen_handlers_mutex);

    if (done) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Timeout waiting for COB-ID %08x", (unsigned)cobid);
    return ESP_ERR_TIMEOUT;
}

static uint32_t canopen_default_pdo_cobid(uint8_t node, canopen_pdo_dir_t dir, uint8_t pdo_num)
{
    static const uint16_t rx_bases[] = { 0x200u, 0x300u, 0x400u, 0x500u };
    static const uint16_t tx_bases[] = { 0x180u, 0x280u, 0x380u, 0x480u };
    const uint16_t *bases = (dir == CANOPEN_PDO_TX) ? tx_bases : rx_bases;
    return (uint32_t)(bases[pdo_num - 1u] + node);
}

static uint16_t canopen_comm_index(canopen_pdo_dir_t dir, uint8_t pdo_num)
{
    return (uint16_t)((dir == CANOPEN_PDO_TX ? 0x1800u : 0x1400u) + (pdo_num - 1u));
}

static uint16_t canopen_map_index(canopen_pdo_dir_t dir, uint8_t pdo_num)
{
    return (uint16_t)((dir == CANOPEN_PDO_TX ? 0x1A00u : 0x1600u) + (pdo_num - 1u));
}

static esp_err_t canopen_pdo_write_u8(uint8_t node, uint16_t index, uint8_t subindex, uint8_t value)
{
    return sdo_download(0x600u + node, index, subindex, &value, sizeof(value));
}

static esp_err_t canopen_pdo_write_u16(uint8_t node, uint16_t index, uint8_t subindex, uint16_t value)
{
    return sdo_download(0x600u + node, index, subindex, &value, sizeof(value));
}

static esp_err_t canopen_pdo_write_u32(uint8_t node, uint16_t index, uint8_t subindex, uint32_t value)
{
    return sdo_download(0x600u + node, index, subindex, &value, sizeof(value));
}

static esp_err_t canopen_pdo_validate_mapping(const uint32_t *mapped, uint8_t mapped_count)
{
    uint16_t total_bits = 0;

    ESP_RETURN_ON_FALSE(mapped_count <= 8u, ESP_ERR_INVALID_ARG, TAG, "mapped_count > 8");
    if (mapped_count == 0u) {
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(mapped != NULL, ESP_ERR_INVALID_ARG, TAG, "mapped is NULL");

    for (uint8_t i = 0; i < mapped_count; ++i) {
        uint8_t nbits = (uint8_t)(mapped[i] & 0xFFu);
        ESP_RETURN_ON_FALSE(nbits > 0u, ESP_ERR_INVALID_ARG, TAG, "mapped entry with 0 bits");
        total_bits = (uint16_t)(total_bits + nbits);
    }

    ESP_RETURN_ON_FALSE(total_bits <= 64u, ESP_ERR_INVALID_ARG, TAG, "PDO payload exceeds 64 bits");
    return ESP_OK;
}

esp_err_t canopen_pdo_configure(uint8_t node,
                                canopen_pdo_dir_t dir,
                                uint8_t pdo_num,
                                const canopen_pdo_cfg_t *cfg)
{
    esp_err_t err;
    uint16_t comm_index;
    uint16_t map_index;
    uint32_t cob_id;

    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");
    ESP_RETURN_ON_FALSE(pdo_num >= 1u && pdo_num <= 4u, ESP_ERR_INVALID_ARG, TAG, "invalid PDO number");
    ESP_RETURN_ON_FALSE(dir == CANOPEN_PDO_RX || dir == CANOPEN_PDO_TX, ESP_ERR_INVALID_ARG, TAG, "invalid PDO dir");

    err = canopen_pdo_validate_mapping(cfg->mapped, cfg->mapped_count);
    if (err != ESP_OK) {
        return err;
    }

    comm_index = canopen_comm_index(dir, pdo_num);
    map_index = canopen_map_index(dir, pdo_num);
    cob_id = cfg->cob_id != 0u ? cfg->cob_id : canopen_default_pdo_cobid(node, dir, pdo_num);

    err = canopen_pdo_write_u32(node, comm_index, 0x01u, cob_id | 0x80000000u);
    if (err != ESP_OK) return err;

    err = canopen_pdo_write_u8(node, map_index, 0x00u, 0u);
    if (err != ESP_OK) return err;

    for (uint8_t i = 0; i < cfg->mapped_count; ++i) {
        err = canopen_pdo_write_u32(node, map_index, (uint8_t)(i + 1u), cfg->mapped[i]);
        if (err != ESP_OK) return err;
    }

    err = canopen_pdo_write_u8(node, map_index, 0x00u, cfg->mapped_count);
    if (err != ESP_OK) return err;

    err = canopen_pdo_write_u8(node, comm_index, 0x02u, cfg->transmission_type);
    if (err != ESP_OK) return err;

    if (dir == CANOPEN_PDO_TX) {
        err = canopen_pdo_write_u16(node, comm_index, 0x03u, cfg->inhibit_time);
        if (err != ESP_OK) return err;
    }

    err = canopen_pdo_write_u32(node, comm_index, 0x01u, cob_id & 0x7FFFFFFFu);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t canopen_pdo_send(uint8_t node,
                           uint8_t rpdo_num,
                           uint32_t cob_id_override,
                           const void *data,
                           size_t len)
{
    twai_message_t msg = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = cob_id_override != 0u ? cob_id_override : canopen_default_pdo_cobid(node, CANOPEN_PDO_RX, rpdo_num),
        .data_length_code = 0,
    };

    ESP_RETURN_ON_FALSE(rpdo_num >= 1u && rpdo_num <= 4u, ESP_ERR_INVALID_ARG, TAG, "invalid RPDO number");
    ESP_RETURN_ON_FALSE(len <= 8u, ESP_ERR_INVALID_ARG, TAG, "PDO payload too large");
    if (len > 0u) {
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is NULL");
        memcpy(msg.data, data, len);
    }
    msg.data_length_code = (uint8_t)len;

    return canopen_send(&msg);
}

esp_err_t canopen_pdo_post(uint8_t node,
                           uint8_t rpdo_num,
                           uint32_t cob_id_override,
                           const void *data,
                           size_t len)
{
    twai_message_t msg = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = cob_id_override != 0u ? cob_id_override : canopen_default_pdo_cobid(node, CANOPEN_PDO_RX, rpdo_num),
        .data_length_code = 0,
    };

    ESP_RETURN_ON_FALSE(rpdo_num >= 1u && rpdo_num <= 4u, ESP_ERR_INVALID_ARG, TAG, "invalid RPDO number");
    ESP_RETURN_ON_FALSE(len <= 8u, ESP_ERR_INVALID_ARG, TAG, "PDO payload too large");
    if (len > 0u) {
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is NULL");
        memcpy(msg.data, data, len);
    }
    msg.data_length_code = (uint8_t)len;

    return canopen_post(&msg);
}

esp_err_t canopen_pdo_subscribe(uint8_t node,
                                uint8_t tpdo_num,
                                uint32_t cob_id_override,
                                canopen_handler_fn fn,
                                void *context,
                                canopen_handler_handle_t *out)
{
    uint32_t cob_id;
    ESP_RETURN_ON_FALSE(tpdo_num >= 1u && tpdo_num <= 4u, ESP_ERR_INVALID_ARG, TAG, "invalid TPDO number");
    ESP_RETURN_ON_FALSE(fn != NULL, ESP_ERR_INVALID_ARG, TAG, "handler is NULL");
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out handle is NULL");

    cob_id = cob_id_override != 0u ? cob_id_override : canopen_default_pdo_cobid(node, CANOPEN_PDO_TX, tpdo_num);
    return canopen_register_handler(cob_id, fn, context, out);
}

void canopen_pdo_payload_clear(canopen_pdo_payload_t *p)
{
    if (p != NULL) {
        memset(p, 0, sizeof(*p));
    }
}

static esp_err_t canopen_pdo_payload_put(canopen_pdo_payload_t *p, const void *src, size_t n)
{
    ESP_RETURN_ON_FALSE(p != NULL, ESP_ERR_INVALID_ARG, TAG, "payload is NULL");
    ESP_RETURN_ON_FALSE(src != NULL, ESP_ERR_INVALID_ARG, TAG, "src is NULL");
    ESP_RETURN_ON_FALSE(p->len + n <= sizeof(p->data), ESP_ERR_INVALID_SIZE, TAG, "payload overflow");

    memcpy(&p->data[p->len], src, n);
    p->len += n;
    return ESP_OK;
}

esp_err_t canopen_pdo_payload_put_u8(canopen_pdo_payload_t *p, uint8_t v)
{
    return canopen_pdo_payload_put(p, &v, sizeof(v));
}

esp_err_t canopen_pdo_payload_put_u16(canopen_pdo_payload_t *p, uint16_t v)
{
    return canopen_pdo_payload_put(p, &v, sizeof(v));
}

esp_err_t canopen_pdo_payload_put_u32(canopen_pdo_payload_t *p, uint32_t v)
{
    return canopen_pdo_payload_put(p, &v, sizeof(v));
}

esp_err_t canopen_pdo_payload_put_i32(canopen_pdo_payload_t *p, int32_t v)
{
    return canopen_pdo_payload_put(p, &v, sizeof(v));
}
