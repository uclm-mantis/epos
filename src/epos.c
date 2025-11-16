#include "epos.h"
#include "epos_types.h"
#include "epos_console.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "linenoise/linenoise.h"
#include <string.h>

#define N_ELEMS(x) (sizeof(x)/sizeof((x)[0]))

typedef struct request_s request_t;
typedef struct response_s response_t;

typedef void (*run_tx_fn_t) (request_t* self);
typedef void (*run_rx_fn_t) (response_t* self, twai_message_t* msg);

struct request_s {
    run_tx_fn_t run;
    twai_message_t* msg;
    uint8_t* value;
    size_t size;
};

struct response_s {
    run_rx_fn_t run;
    uint32_t cobid;
    uint8_t* value;
    size_t size;
};

static const char* TAG = "EPOS";
TickType_t maxDelay = pdMS_TO_TICKS(3000);
bool enable_dump_msg = false;

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t sdo_sem;
static SemaphoreHandle_t nmt_sem;
static SemaphoreHandle_t done_sem;

static void dump_msg(const char* info, twai_message_t* msg)
{
    if (enable_dump_msg) {
        uint8_t* d = msg->data;
        ESP_LOGI("EPOS", "%s %08x (%d bytes) %02x %02x %02x %02x %02x %02x %02x %02x",
                 info, (unsigned)msg->identifier, (int)msg->data_length_code, 
                 d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    }
}



/*  Asynchronous CANopen handlers

    De momento solo ponemos recepción de mensajes asíncronos. 
    - Receive PDO
    - NMT Error Control
    - NMT Bootup
    - EMCY

    En realidad se puede usar para caulquier otro protocolo asíncrono (e.g. NMT bootup, NMT heartbeat, EMCY, etc.)
*/
typedef struct canopen_handler_entry {
    canopen_handler_fn handler;
    uint32_t cobid;
    void* context;                  // Datos de contexto asociados
    struct canopen_handler_entry* next; // Siguiente en la lista
} canopen_handler_entry_t;

static canopen_handler_entry_t* canopen_handlers = NULL;

static void epos_receive_canopen(twai_message_t* msg) {
    for(canopen_handler_entry_t* current = canopen_handlers; current != NULL; current = current->next) {
        if (current->cobid == msg->identifier) {
            current->handler(current->cobid, msg->data, current->context);
            dump_msg("Receive CO", msg);
            return;
        }
    }
    dump_msg("Skipping", msg);
}

esp_err_t epos_register_canopen_handler(uint32_t cobid, canopen_handler_fn handler_fn, void* context) 
{
    ESP_RETURN_ON_FALSE((handler_fn != NULL), -1, TAG, "Invalid CANopen handler for %08x", (int)cobid);
    canopen_handler_entry_t* new_entry = (canopen_handler_entry_t*) malloc(sizeof(canopen_handler_entry_t));
    ESP_RETURN_ON_FALSE((new_entry != NULL), -1, TAG, "Memory alloc failure");

    new_entry->cobid = cobid;
    new_entry->handler = handler_fn;
    new_entry->context = context;
    new_entry->next = canopen_handlers;
    canopen_handlers = new_entry;

    return ESP_OK;
}

esp_err_t epos_unregister_canopen_handler(uint32_t cobid) 
{
    canopen_handler_entry_t* current = canopen_handlers;
    canopen_handler_entry_t* prev = NULL;

    while (current != NULL) {
        if (current->cobid == cobid) {
            if (prev == NULL) {
                canopen_handlers = current->next;
            } else {
                prev->next = current->next;
            }
            free(current);
            return ESP_OK;
        }
        prev = current;
        current = current->next;
    }
    return ESP_FAIL;
}


/*  Protocolo SDO_DOWNLOAD.
    Debe enviar la petición y esperar la respuesta, que no lleva carga útil.

    La petición de SDO es con COB-ID 0x600 + n, la respuesta 0x580 + n.
    Permitimos petición sin respuesta transcurrido cierto tiempo.

    TODO: si no es expedited debe encolar un SDO_SEGMENT_REQ en response.
    En self debe llevar cuenta de lo que queda por transmitir. Cuando transmita
    todo debe poner c a 1.
 */
static void sdo_download_response(response_t* self, twai_message_t* msg) 
{
    dump_msg("Receive", msg);
    SDO_download_resp_t* resp = (SDO_download_resp_t*) msg->data;
    if (resp->scs != SDO_SCS_DOWNLOAD) {
        dump_msg("Wrong response code", msg);
    }
    xSemaphoreGive(sdo_sem); // debería ejecutarse solo en el último segmento+
}

static void sdo_download_segment_response(response_t* self, twai_message_t* msg);

static void sdo_download_segment_request(request_t* self) 
{
    response_t resp = { .run = sdo_download_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .size = self->size };
    xQueueSend(rx_task_queue, &resp, portMAX_DELAY);
    dump_msg("Tx Seg", self->msg);
    twai_transmit(self->msg, portMAX_DELAY);
}

static void sdo_download_segment_response(response_t* self, twai_message_t* msg) 
{
    dump_msg("Rx Seg", msg);
    static twai_message_t req_msg;
    req_msg = (twai_message_t) { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };
    if (self->size > 0) { // more segments remaining
        SDO_download_seg_resp_t* resp = (SDO_download_seg_resp_t*) msg->data;
        SDO_download_seg_req_t* req_payload = (SDO_download_seg_req_t*) req_msg.data;
        size_t n = (self->size < 7 ? self->size : 7);
        *req_payload = (SDO_download_seg_req_t){ .ccs = SDO_CCS_DOWNLOAD_SEG, 
                                                 .t = (resp->scs != SDO_SCS_DOWNLOAD && !resp->t), // not first segment and not previous toggle bit
                                                 .c = (n == self->size) };
        memcpy(req_payload->seg_data, self->value, n);
        request_t req = { .run = sdo_download_segment_request, .msg = &req_msg, .value = self->value + n, .size = self->size - n };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
    else { // confirmation of last segment
        xSemaphoreGive(sdo_sem);
    }
}

static void sdo_download_request(request_t* self) 
{
    SDO_download_req_t* payload = (SDO_download_req_t*) self->msg->data;
    if (payload->e) { // expedited
        response_t resp = { .run =  sdo_download_response, .cobid = self->msg->identifier - 0x600 + 0x580 };
        xQueueSend(rx_task_queue, &resp, portMAX_DELAY);
    }
    else {
        response_t resp = { .run =  sdo_download_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .size = self->size };
        xQueueSend(rx_task_queue, &resp, portMAX_DELAY);
    }
    dump_msg("Transmit", self->msg);
    twai_transmit(self->msg, portMAX_DELAY);
}

esp_err_t sdo_download(uint32_t id, uint16_t index, uint8_t subindex, void* value, size_t size) 
{
    static twai_message_t msg;
    msg = (twai_message_t){ .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = id, .data_length_code = 8 };
    SDO_download_req_t* payload = (SDO_download_req_t*) msg.data;
    if (size <= 4) { // may be expedited
        *payload = (SDO_download_req_t){ .s = 1, .e = 1, .n = 4 - size, .x = 0, .ccs = SDO_CCS_DOWNLOAD, .index = index, .subindex = subindex};
        memcpy(payload->d, value, size);
    }
    else { // must be segmented
        *payload = (SDO_download_req_t){ .s = 1, .e = 0, .n = 0, .x = 0, .ccs = SDO_CCS_DOWNLOAD, .index = index, .subindex = subindex, .dsize = size };
    }
    request_t req = { .run = sdo_download_request, .msg = &msg, .value = value };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (xSemaphoreTake(sdo_sem, maxDelay) != pdTRUE) {
        response_t resp;
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        xQueueReceive(rx_task_queue, &resp, portMAX_DELAY); // remove request
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*  Protocolo SDO_UPLOAD.
    Debe enviar la petición y esperar la respuesta, que lleva la carga útil.

    La petición de SDO es con COB-ID 0x600 + n, la respuesta 0x580 + n.
    Permitimos petición sin respuesta transcurrido cierto tiempo, generando error.

    TODO: si no es expedited debe encolar un SDO_SEGMENT_REQ en response.
    En self debe llevar cuenta de lo que queda por transmitir. Cuando transmita
    todo debe poner c a 1.
 */
static void sdo_upload_segment_request(request_t* self);

static void sdo_upload_segment_response(response_t* self, twai_message_t* msg) 
{
    dump_msg("Rx Seg", msg);
    SDO_upload_seq_resp_t* payload = (SDO_upload_seq_resp_t*) msg->data;
    size_t n = 7 - payload->n;
    memcpy(self->value, payload->seg_data, n);
    if (payload->c) {
        xSemaphoreGive(sdo_sem);
    }
    else {
        static twai_message_t req_msg;
        req_msg = (twai_message_t) { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };
        SDO_upload_seq_req_t* req_payload = (SDO_upload_seq_req_t*) req_msg.data;
        *req_payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = !payload->t, .reserved = {0} };
        request_t req = { .run = sdo_upload_segment_request, .msg = &req_msg, .value = self->value + n };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
}

static void sdo_upload_segment_request(request_t* self) 
{
    response_t resp = { .run = sdo_upload_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value };
    xQueueSend(rx_task_queue, &resp, portMAX_DELAY);
    dump_msg("Tx Seg", self->msg);
    twai_transmit(self->msg, portMAX_DELAY);
}

static void sdo_upload_response(response_t* self, twai_message_t* msg) 
{
    dump_msg("Receive", msg);
    SDO_upload_resp_t* payload = (SDO_upload_resp_t*) msg->data;
    size_t n = 4 - payload->n;
    if (n > 0)
        memcpy(self->value, payload->d, 4 - payload->n);
    if (payload->e) {
        xSemaphoreGive(sdo_sem);
    }
    else {
        static twai_message_t req_msg;
        req_msg = (twai_message_t){ .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };
        SDO_upload_seq_req_t* payload = (SDO_upload_seq_req_t*) req_msg.data;
        *payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = 0, .reserved = {0} };
        request_t req = { .run = sdo_upload_segment_request, .msg = &req_msg, .value =self->value + n };
        xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    }
}

static void sdo_upload_request(request_t* self) 
{
    response_t resp = { .run = sdo_upload_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value };
    xQueueSend(rx_task_queue, &resp, portMAX_DELAY);
    dump_msg("Transmit", self->msg);
    twai_transmit(self->msg, portMAX_DELAY);
}

esp_err_t sdo_upload(uint32_t id, uint16_t index, uint8_t subindex, void* ret) 
{
    static twai_message_t msg;
    msg = (twai_message_t) { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = id, .data_length_code = 8 };
    SDO_upload_req_t* payload = (SDO_upload_req_t*) msg.data;
    *payload = (SDO_upload_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD, .index = index, .subindex = subindex, .reserved = {0} };

    request_t req = { .run = sdo_upload_request, .msg = &msg, .value = ret };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (xSemaphoreTake(sdo_sem, maxDelay) != pdTRUE) {
        response_t resp;
        ESP_LOGI(TAG, "Peer does not seem to be available, unconfirmed request");
        xQueueReceive(rx_task_queue, &resp, portMAX_DELAY); // remove request
        return ESP_FAIL;
   }
   return ESP_OK;
}

/*  Protocolo NMT

               +------------(1)------------------------------------------------+
               |  +---------(1)-----------------------------+                  |
               |  |  +------(1)---------+                   |                  |
               v  v  v                  |                   |                  |
    (*) --> Initialisation -(2)-> Preoperational -(3)-> Operational -(4)-> Stopped
                                       ^   |                 ^             | ^ |
                                       |   |                 +-------(3)---+ | |
                                       |   +-------------------------(4)-----+ |
                                       +-----------------------------(5)-------+

    (1) reset node / reset comm
    (2) implicit, generates bootup NMT message
    (3) start
    (4) stop
    (5) preop

 */
static void nmt_request(request_t* self) 
{
    dump_msg("Transmit", self->msg);
    twai_transmit(self->msg, portMAX_DELAY);
    xSemaphoreGive(nmt_sem);
}

esp_err_t nmt(uint8_t cs, uint8_t n) 
{
    twai_message_t msg = { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = 0, .data_length_code = 2, .data = { cs, n, 0, 0 } };
    request_t req = { .run = nmt_request, .msg = &msg };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);
    if (xSemaphoreTake(nmt_sem, maxDelay) != pdTRUE) {
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

esp_err_t nmt_enter_preoperational(uint8_t node) { return nmt(NMT_ENTER_PREOPERATIONAL, node); }
esp_err_t nmt_reset_communication(uint8_t node)  { return nmt(NMT_RESET_COMMUNICATION, node); }
esp_err_t nmt_reset_node(uint8_t node)           { return nmt(NMT_RESET_NODE, node); }
esp_err_t nmt_start_remote_node(uint8_t node)    { return nmt(NMT_START_REMOTE_NODE, node); }
esp_err_t nmt_stop_remote_node(uint8_t node)     { return nmt(NMT_STOP_REMOTE_NODE, node); }


/* Para salir de los hilos de transmisión y recepción */
static void epos_req_done_run(request_t* self) { vTaskDelete(NULL); } 
static void epos_resp_done_run(response_t* self, twai_message_t* msg) { vTaskDelete(NULL); } 

void epos_done()
{
    request_t req = { .run = &epos_req_done_run };
    xQueueSend(tx_task_queue, &req, portMAX_DELAY);

    response_t resp = { .run = &epos_resp_done_run };
    xQueueSend(rx_task_queue, &resp, portMAX_DELAY);

    xSemaphoreGive(done_sem);
    vTaskDelete(NULL); // exit calling task
}


static void epos_transmit_task(void *arg)
{
    for (;;) {
        request_t req;
        xQueueReceive(tx_task_queue, &req, portMAX_DELAY);
        req.run(&req);
    }
}

static void epos_receive_task(void *arg)
{
    ESP_LOGI(TAG, "Receiver task");
    static twai_message_t msg;
    for (;;) {
        if (ESP_OK != twai_receive(&msg, maxDelay)) continue;
        response_t resp; // si hay petición pendiente, atender
        if (xQueuePeek(rx_task_queue, &resp, 0) == pdTRUE) {
            if (msg.identifier == resp.cobid) {
                if (xQueueReceive(rx_task_queue, &resp, 0) != pdTRUE)
                    ESP_LOGE(TAG, "RX queue was empty after successful peek!");
                resp.run(&resp, &msg);
                continue;
            }
        }
        epos_receive_canopen(&msg);
    }
}

/*
    transmit and receive tasks are always needed for any EPOS interaction

    On the other side, epos_console_task is only one of the many controlling tasks you may
    use for a specific application. Remove it if you need more memory and controlling operations
    are already well defined
 */

void epos_console_task(void *arg);

static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(DEFAULT_CAN_TX, DEFAULT_CAN_RX, TWAI_MODE_NORMAL);

void epos_set_tx_rx_pins(gpio_num_t tx, gpio_num_t rx)
{
    g_config.tx_io = tx;
    g_config.rx_io = rx;
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t epos_initialize(bool activate_console)
{
    initialize_nvs();

    ESP_RETURN_ON_ERROR(twai_driver_install(&g_config, &t_config, &f_config), TAG, "Unable to install TWAI driver");
    ESP_RETURN_ON_ERROR(twai_start(), TAG, "Unable to start TWAI");

    tx_task_queue = xQueueCreate(1, sizeof(request_t));
    rx_task_queue = xQueueCreate(1, sizeof(response_t));
 
    sdo_sem = xSemaphoreCreateBinary();
    nmt_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(epos_receive_task, "receive",   4096, NULL,  8, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(epos_transmit_task, "transmit", 4096, NULL,  9, NULL, tskNO_AFFINITY);
    if (activate_console)
        xTaskCreatePinnedToCore(epos_console_task, "console",   4096, NULL, 10, NULL, tskNO_AFFINITY);
    return ESP_OK;
}

esp_err_t epos_wait_done()
{
    xSemaphoreTake(done_sem, portMAX_DELAY);

    ESP_RETURN_ON_ERROR(twai_stop(), TAG, "Unable to stop TWAI");
    ESP_RETURN_ON_ERROR(twai_driver_uninstall(), TAG, "Unable to uninstall TWAI driver");

    vQueueDelete(rx_task_queue);
    vQueueDelete(tx_task_queue);
    vSemaphoreDelete(sdo_sem);
    vSemaphoreDelete(nmt_sem);
    vSemaphoreDelete(done_sem);
    return ESP_OK;
}


typedef struct {
    uint32_t cobid;
    SemaphoreHandle_t semaphore;
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


static void epos_signal_cobid(uint32_t cobid, void* data, void* ret) 
{
    for (int i = 0; i < N_ELEMS(cobid_table); i++) {
        if (cobid_table[i].in_use && cobid_table[i].cobid == cobid) {
            if (ret) memcpy(ret, data, 8); // copy CANopen payload
            xSemaphoreGive(cobid_table[i].semaphore);
            cobid_table[i].in_use = false;
            return;
        }
    }
}

esp_err_t epos_wait_until(uint32_t cobid, void* ret) 
{
    for (int i = 0; i < N_ELEMS(cobid_table); ++i) {
        if (!cobid_table[i].in_use) {
            if (cobid_table[i].semaphore == NULL) {
                cobid_table[i].semaphore = xSemaphoreCreateBinary();
                if (cobid_table[i].semaphore == NULL) {
                    ESP_LOGE(TAG, "Failed to create semaphore for COB-ID %08x", (unsigned)cobid);
                    return ESP_ERR_NO_MEM;
                }
            }
            cobid_table[i].cobid = cobid;
            cobid_table[i].in_use = true;

            epos_register_canopen_handler(cobid, epos_signal_cobid, ret);
            BaseType_t done = xSemaphoreTake(cobid_table[i].semaphore, maxDelay);
            epos_unregister_canopen_handler(cobid);

            if (done) return ESP_OK;
            ESP_LOGE(TAG, "Timeout waiting for COB-ID %08x", (unsigned)cobid);
            return ESP_ERR_TIMEOUT;
        }
    }

    ESP_LOGE(TAG, "No free entries in COB-ID table for %08x", (unsigned)cobid);
    return ESP_ERR_NO_MEM;
}
