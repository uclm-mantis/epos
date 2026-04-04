#include "canopen.h"
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

typedef struct request_s request_t;
typedef struct response_s response_t;

typedef void (*run_tx_fn_t) (request_t* self);
typedef void (*run_rx_fn_t) (response_t* self, twai_message_t* msg);

struct request_s {
    run_tx_fn_t run;
    twai_message_t msg;
    uint8_t* value;
    size_t size;
    TaskHandle_t waiter;
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
static twai_node_handle_t twai_node;

#define CANOPEN_INTERNAL_STOP_ID   0xFFFFFFFFu

static portMUX_TYPE done_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t done_waiter_task = NULL;
static bool shutdown_requested = false;

static TaskHandle_t dispatch_task_handle;
static TaskHandle_t tx_task_handle;

static volatile bool twai_bus_off = false;
static TickType_t twai_last_bus_off_log = 0;
static TickType_t twai_last_recover_attempt = 0;
static TickType_t twai_recover_backoff = pdMS_TO_TICKS(250);

static void canopen_note_bus_off(void)
{
    twai_bus_off = true;
}

static void canopen_clear_bus_off(void)
{
    twai_bus_off = false;
}

static void canopen_log_bus_off_once(void)
{
    TickType_t now = xTaskGetTickCount();
    if ((now - twai_last_bus_off_log) >= pdMS_TO_TICKS(1000)) {
        twai_last_bus_off_log = now;
        ESP_LOGE(TAG, "TWAI node is bus-off");
    }
}

static void canopen_poll_bus_state(void)
{
    if (twai_node == NULL || !twai_bus_off) {
        return;
    }

    twai_node_status_t status = {0};
    if (twai_node_get_info(twai_node, &status, NULL) == ESP_OK) {
        if (status.state != TWAI_ERROR_BUS_OFF) {
            canopen_clear_bus_off();
        }
    }
}

static void canopen_request_recovery_if_needed(void)
{
    TickType_t now;

    if (twai_node == NULL || !twai_bus_off) {
        return;
    }

    now = xTaskGetTickCount();
    if ((now - twai_last_recover_attempt) < twai_recover_backoff) {
        return;
    }
    twai_last_recover_attempt = now;

    esp_err_t err = twai_node_recover(twai_node);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "TWAI recovery started");
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "twai_node_recover failed: %s", esp_err_to_name(err));
    }
}

static int canopen_timeout_ms(TickType_t ticks)
{
    if (ticks == portMAX_DELAY) {
        return -1;
    }
    uint32_t ms = pdTICKS_TO_MS(ticks);
    if (ms == 0 && ticks > 0) {
        ms = 1;
    }
    return (int)ms;
}

static void canopen_msg_to_frame(const twai_message_t *msg, twai_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->header.id = msg->identifier;
    frame->header.ide = msg->extd ? 1u : 0u;
    frame->header.rtr = msg->rtr ? 1u : 0u;
    frame->header.fdf = 0;
    frame->header.brs = 0;
    frame->header.dlc = msg->data_length_code;
    frame->buffer = (uint8_t *)msg->data;
    frame->buffer_len = msg->data_length_code;
}

static void canopen_frame_to_msg(const twai_frame_t *frame, twai_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));
    msg->identifier = frame->header.id;
    msg->extd = frame->header.ide != 0;
    msg->rtr = frame->header.rtr != 0;
    msg->data_length_code = (uint8_t)frame->buffer_len;
    if (msg->data_length_code > sizeof(msg->data)) {
        msg->data_length_code = sizeof(msg->data);
    }
    if (frame->buffer != NULL && msg->data_length_code > 0) {
        memcpy(msg->data, frame->buffer, msg->data_length_code);
    }
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

static void canopen_send_now(request_t* self)
{
    twai_frame_t frame;
    esp_err_t err;

    if (twai_node == NULL) {
        ESP_LOGE(TAG, "TWAI node not initialized");
        return;
    }

    canopen_poll_bus_state();
    if (twai_bus_off) {
        canopen_request_recovery_if_needed();
        return;
    }

    dump_msg("Transmit", &self->msg);
    canopen_msg_to_frame(&self->msg, &frame);
    err = twai_node_transmit(twai_node, &frame, canopen_timeout_ms(max_delay));
    if (err != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            canopen_note_bus_off();
            canopen_request_recovery_if_needed();
            canopen_log_bus_off_once();
        } else {
            ESP_LOGE(TAG, "twai_node_transmit failed: %s", esp_err_to_name(err));
        }
        return;
    }

    err = twai_node_transmit_wait_all_done(twai_node, canopen_timeout_ms(max_delay));
    if (err != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            canopen_note_bus_off();
            canopen_request_recovery_if_needed();
            canopen_log_bus_off_once();
        } else {
            ESP_LOGE(TAG, "twai_node_transmit_wait_all_done failed: %s", esp_err_to_name(err));
        }
    }
}

static void canopen_send_notify(request_t *self)
{
    canopen_send_now(self);
    if (self->waiter != NULL) {
        xTaskNotifyGive(self->waiter);
    }
}

esp_err_t canopen_post(const twai_message_t *msg)
{
    request_t req;

    ESP_RETURN_ON_FALSE(msg != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL msg");
    ESP_RETURN_ON_FALSE(tx_task_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "CANopen not initialized");

    canopen_poll_bus_state();
    if (twai_bus_off) {
        canopen_request_recovery_if_needed();
        return ESP_ERR_INVALID_STATE;
    }

    req = (request_t){
        .run = canopen_send_now,
        .msg = *msg,
        .value = NULL,
        .size = 0,
        .waiter = NULL,
    };

    return xQueueSend(tx_task_queue, &req, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t canopen_send(const twai_message_t *msg)
{
    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    request_t req;

    ESP_RETURN_ON_FALSE(msg != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL msg");
    ESP_RETURN_ON_FALSE(tx_task_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "CANopen not initialized");

    canopen_poll_bus_state();
    if (twai_bus_off) {
        canopen_request_recovery_if_needed();
        canopen_log_bus_off_once();
        return ESP_ERR_INVALID_STATE;
    }

    (void) ulTaskNotifyTake(pdTRUE, 0);

    req = (request_t){
        .run = canopen_send_notify,
        .msg = *msg,
        .value = NULL,
        .size = 0,
        .waiter = waiter,
    };

    if (xQueueSend(tx_task_queue, &req, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (ulTaskNotifyTake(pdTRUE, max_delay) != pdTRUE) {
        ESP_LOGI(TAG, "Sender thread is blocked, unable to transmit CAN frame");
        return ESP_FAIL;
    }

    return ESP_OK;
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

static void sdo_download_segment_request(request_t* self)
{
    response_t resp = { .run = sdo_download_segment_response, .cobid = self->msg.identifier - 0x600 + 0x580, .value = self->value, .size = self->size, .waiter = self->waiter };
    resp.match = match_cobid_only;
    esp_err_t err = canopen_add_pending_response(&resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No free pending response slots");
        return;
    }
    canopen_send_now(self);
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
        request_t req = { .run = sdo_download_segment_request, .msg = req_msg, .value = self->value + n, .size = self->size - n, .waiter = self->waiter };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    } else {
        xTaskNotifyGive(self->waiter);
    }
}

static void sdo_download_request(request_t* self)
{
    SDO_download_req_t* payload = (SDO_download_req_t*) self->msg.data;
    if (payload->e) {
        response_t resp = { .run =  sdo_download_response, .cobid = self->msg.identifier - 0x600 + 0x580, .waiter = self->waiter };
        resp.match = match_cobid_only;
        esp_err_t err = canopen_add_pending_response(&resp);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "No free pending response slots");
            return;
        }
    } else {
        response_t resp = { .run =  sdo_download_segment_response, .cobid = self->msg.identifier - 0x600 + 0x580, .value = self->value, .size = self->size, .waiter = self->waiter };
        resp.match = match_cobid_only;
        esp_err_t err = canopen_add_pending_response(&resp);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "No free pending response slots");
            return;
        }
    }
    canopen_send_now(self);
}

esp_err_t sdo_download(uint32_t id, uint16_t index, uint8_t subindex, void* value, size_t size)
{
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
    request_t req = { .run = sdo_download_request, .msg = msg, .value = value, .size = size, .waiter = waiter };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (ulTaskNotifyTake(pdTRUE, max_delay) != pdTRUE) {
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void sdo_upload_segment_request(request_t* self);

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
        request_t req = { .run = sdo_upload_segment_request, .msg = req_msg, .value = self->value + n, .waiter = self->waiter };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
}

static void sdo_upload_segment_request(request_t* self)
{
    response_t resp = { .run = sdo_upload_segment_response, .cobid = self->msg.identifier - 0x600 + 0x580, .value = self->value, .waiter = self->waiter };
    resp.match = match_cobid_only;
    esp_err_t err = canopen_add_pending_response(&resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No free pending response slots");
        return;
    }
    canopen_send_now(self);
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
        request_t req = { .run = sdo_upload_segment_request, .msg = req_msg, .value = self->value + n, .waiter = self->waiter };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
}

static void sdo_upload_request(request_t* self)
{
    response_t resp = { .run = sdo_upload_response, .cobid = self->msg.identifier - 0x600 + 0x580, .value = self->value, .waiter = self->waiter };
    resp.match = match_cobid_only;
    esp_err_t err = canopen_add_pending_response(&resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No free pending response slots");
        return;
    }
    canopen_send_now(self);
}

esp_err_t sdo_upload(uint32_t id, uint16_t index, uint8_t subindex, void* ret)
{
    twai_message_t msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = id, .data_length_code = 8 };
    SDO_upload_req_t* payload = (SDO_upload_req_t*) msg.data;
    *payload = (SDO_upload_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD, .index = index, .subindex = subindex, .reserved = {0} };

    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0);
    request_t req = { .run = sdo_upload_request, .msg = msg, .value = ret, .waiter = waiter };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (ulTaskNotifyTake(pdTRUE, max_delay) != pdTRUE) {
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        return ESP_FAIL;
   }
   return ESP_OK;
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

static bool canopen_twainode_rx_done_cb(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void) edata;
    (void) user_ctx;

    uint8_t rx_buf[8] = {0};
    twai_frame_t frame = {
        .buffer = rx_buf,
        .buffer_len = sizeof(rx_buf),
    };
    twai_message_t msg;
    BaseType_t high_task_wakeup = pdFALSE;

    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) {
        return false;
    }

    canopen_frame_to_msg(&frame, &msg);
    canopen_clear_bus_off();
    if (xQueueSendFromISR(rx_msg_queue, &msg, &high_task_wakeup) != pdTRUE) {
        return false;
    }

    return high_task_wakeup == pdTRUE;
}

static void canopen_transmit_task(void *arg)
{
    for (;;) {
        request_t req;
        if (xQueueReceive(tx_task_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (req.run == NULL) break;
        req.run(&req);
    }
    vTaskDelete(NULL);
}

static void canopen_dispatch_task(void *arg)
{
    twai_message_t msg;
    response_t resp;

    for (;;) {
        if (xQueueReceive(rx_msg_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (msg.identifier == CANOPEN_INTERNAL_STOP_ID) break;
        if (canopen_take_pending_response(&msg, &resp)) {
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
    twai_onchip_node_config_t node_cfg = {0};
    twai_mask_filter_config_t filter_cfg = {0};
    twai_event_callbacks_t callbacks = {
        .on_rx_done = canopen_twainode_rx_done_cb,
    };

    if (cfg == NULL)
        cfg = &default_cfg;

    max_delay = pdMS_TO_TICKS(cfg->max_delay_ms);
    enable_dump_msg = cfg->enable_dump_msg;

    rx_msg_queue = xQueueCreate(16, sizeof(twai_message_t));
    tx_task_queue = xQueueCreate(8, sizeof(request_t));
    canopen_handlers_mutex = xSemaphoreCreateMutex();
    cobid_mutex = xSemaphoreCreateMutex();

    ESP_RETURN_ON_FALSE(rx_msg_queue != NULL && tx_task_queue != NULL &&
                        canopen_handlers_mutex != NULL && cobid_mutex != NULL,
                        ESP_ERR_NO_MEM, TAG, "Unable to allocate CANopen resources");

    node_cfg.io_cfg.tx = (gpio_num_t) cfg->can_tx_pin;
    node_cfg.io_cfg.rx = (gpio_num_t) cfg->can_rx_pin;
    node_cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    node_cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    node_cfg.bit_timing.bitrate = 1000000;
    node_cfg.tx_queue_depth = 8;
    node_cfg.fail_retry_cnt = -1;
    node_cfg.flags.no_receive_rtr = 1;

    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&node_cfg, &twai_node), TAG, "Unable to create TWAI node");
    ESP_RETURN_ON_ERROR(twai_node_register_event_callbacks(twai_node, &callbacks, NULL), TAG, "Unable to register TWAI callbacks");

    filter_cfg.id = 0;
    filter_cfg.mask = 0;
    filter_cfg.is_ext = 0;
    filter_cfg.no_classic = 0;
    filter_cfg.no_fd = 1;
    ESP_RETURN_ON_ERROR(twai_node_config_mask_filter(twai_node, 0, &filter_cfg), TAG, "Unable to configure TWAI filter");
    ESP_RETURN_ON_ERROR(twai_node_enable(twai_node), TAG, "Unable to enable TWAI node");

    taskENTER_CRITICAL(&done_mux);
    shutdown_requested = false;
    done_waiter_task = NULL;
    taskEXIT_CRITICAL(&done_mux);

    twai_bus_off = false;
    twai_last_bus_off_log = 0;
    twai_last_recover_attempt = 0;

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

    if (twai_node != NULL) {
        ESP_RETURN_ON_ERROR(twai_node_disable(twai_node), TAG, "Unable to disable TWAI node");
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

    if (twai_node != NULL) {
        ESP_RETURN_ON_ERROR(twai_node_delete(twai_node), TAG, "Unable to delete TWAI node");
        twai_node = NULL;
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
    dispatch_task_handle = NULL;
    tx_task_handle = NULL;
    taskEXIT_CRITICAL(&done_mux);

    twai_bus_off = false;
    twai_last_bus_off_log = 0;
    twai_last_recover_attempt = 0;

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
