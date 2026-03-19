#include "canopen.h"
#include "canopen_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/twai.h"

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
TickType_t maxDelay = pdMS_TO_TICKS(3000);
bool enable_dump_msg = false;

static QueueHandle_t rx_msg_queue;
static QueueHandle_t tx_task_queue;

#define CANOPEN_INTERNAL_STOP_ID   0xFFFFFFFFu

static portMUX_TYPE done_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t done_waiter_task = NULL;
static bool shutdown_requested = false;

static TaskHandle_t rx_task_handle;
static TaskHandle_t dispatch_task_handle;
static TaskHandle_t tx_task_handle;

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
        ESP_LOGI("CANOPEN", "%s %08x (%d bytes) %02x %02x %02x %02x %02x %02x %02x %02x",
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
    dump_msg("Tx Seg", &self->msg);
    twai_transmit(&self->msg, portMAX_DELAY);
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
    dump_msg("Transmit", &self->msg);
    twai_transmit(&self->msg, portMAX_DELAY);
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
    if (ulTaskNotifyTake(pdTRUE, maxDelay) != pdTRUE) {
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
    dump_msg("Tx Seg", &self->msg);
    twai_transmit(&self->msg, portMAX_DELAY);
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
    dump_msg("Transmit", &self->msg);
    twai_transmit(&self->msg, portMAX_DELAY);
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
    if (ulTaskNotifyTake(pdTRUE, maxDelay) != pdTRUE) {
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        return ESP_FAIL;
   }
   return ESP_OK;
}

static void nmt_request(request_t* self)
{
    dump_msg("Transmit", &self->msg);
    twai_transmit(&self->msg, portMAX_DELAY);
    xTaskNotifyGive(self->waiter);
}

esp_err_t nmt(uint8_t cs, uint8_t n)
{
    twai_message_t msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = 0, .data_length_code = 2, .data = { cs, n, 0, 0 } };
    TaskHandle_t waiter = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0);
    request_t req = { .run = nmt_request, .msg = msg, .waiter = waiter };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (ulTaskNotifyTake(pdTRUE, maxDelay) != pdTRUE) {
        ESP_LOGI(TAG, "Sender thread is blocked, unable to transmit NMT request");
        return ESP_FAIL;
    }
    return ESP_OK;
}

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_GETTER(r,id,sid,t)
#include "object_dictionary.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SETTER(w,id,sid,t)
#include "object_dictionary.h"
#undef OBJ


void canopen_done(void)
{
    taskENTER_CRITICAL(&done_mux);
    shutdown_requested = true;
    TaskHandle_t waiter = done_waiter_task;
    taskEXIT_CRITICAL(&done_mux);

    if (waiter != NULL) {
        xTaskNotifyGive(waiter);
    }
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

static void canopen_receive_task(void *arg)
{
    twai_message_t msg;

    for (;;) {
        esp_err_t err = twai_receive(&msg, portMAX_DELAY);

        bool stop;
        taskENTER_CRITICAL(&done_mux);
        stop = shutdown_requested;
        taskEXIT_CRITICAL(&done_mux);

        if (stop) break;

        if (err != ESP_OK) {
            continue;
        }

        if (xQueueSend(rx_msg_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "RX queue full, dropping CAN frame");
        }
    }
    vTaskDelete(NULL);
}

static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(DEFAULT_CAN_TX, DEFAULT_CAN_RX, TWAI_MODE_NORMAL);

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t canopen_initialize(const canopen_init_cfg_t *cfg)
{
    static canopen_init_cfg_t default_cfg = CANOPEN_INIT_DEFAULT();
    if (cfg == NULL)
        cfg = &default_cfg;
    g_config.tx_io = (gpio_num_t) cfg->can_tx_pin;
    g_config.rx_io = (gpio_num_t) cfg->can_rx_pin;

    initialize_nvs();

    ESP_RETURN_ON_ERROR(twai_driver_install(&g_config, &t_config, &f_config), TAG, "Unable to install TWAI driver");
    ESP_RETURN_ON_ERROR(twai_start(), TAG, "Unable to start TWAI");

    rx_msg_queue = xQueueCreate(16, sizeof(twai_message_t));
    tx_task_queue = xQueueCreate(8, sizeof(request_t));
    canopen_handlers_mutex = xSemaphoreCreateMutex();
    cobid_mutex = xSemaphoreCreateMutex();

    taskENTER_CRITICAL(&done_mux);
    shutdown_requested = false;
    done_waiter_task = NULL;
    taskEXIT_CRITICAL(&done_mux);

    xTaskCreatePinnedToCore(canopen_receive_task, "receive",   4096, NULL,  8, &rx_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(canopen_dispatch_task, "dispatch", 4096, NULL, 8, &dispatch_task_handle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(canopen_transmit_task, "transmit", 4096, NULL,  9, &tx_task_handle, tskNO_AFFINITY);

    return ESP_OK;
}

esp_err_t canopen_wait_done(void)
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

    ESP_RETURN_ON_ERROR(twai_stop(), TAG, "Unable to stop TWAI");

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

    ESP_RETURN_ON_ERROR(twai_driver_uninstall(), TAG, "Unable to uninstall TWAI driver");

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

    done = ulTaskNotifyTake(pdTRUE, maxDelay);

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
