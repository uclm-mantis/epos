import sys
import os

filepath = r"d:\git\prj\epos\src\epos.c"

with open(filepath, "r", encoding="utf-8") as f:
    content = f.read()

replacements = [
    (
        "struct request_s {\n    run_tx_fn_t run;\n    twai_message_t* msg;\n    uint8_t* value;\n    size_t size;\n};",
        "struct request_s {\n    run_tx_fn_t run;\n    twai_message_t* msg;\n    uint8_t* value;\n    size_t size;\n    TaskHandle_t task;\n};"
    ),
    (
        "struct response_s {\n    run_rx_fn_t run;\n    uint32_t cobid;\n    uint8_t* value;\n    size_t size;\n};",
        "struct response_s {\n    run_rx_fn_t run;\n    uint32_t cobid;\n    uint8_t* value;\n    size_t size;\n    TaskHandle_t task;\n};"
    ),
    (
        "static QueueHandle_t tx_task_queue;\nstatic QueueHandle_t rx_task_queue;\nstatic SemaphoreHandle_t sdo_sem;\nstatic SemaphoreHandle_t nmt_sem;\nstatic SemaphoreHandle_t done_sem;",
        "static QueueHandle_t tx_task_queue;\nstatic QueueHandle_t rx_task_queue;\nstatic TaskHandle_t done_task_handle = NULL;"
    ),
    (
        "    if (resp->scs != SDO_SCS_DOWNLOAD) {\n        dump_msg(\"Wrong response code\", msg);\n    }\n    xSemaphoreGive(sdo_sem); // debería ejecutarse solo en el último segmento+",
        "    if (resp->scs != SDO_SCS_DOWNLOAD) {\n        dump_msg(\"Wrong response code\", msg);\n    }\n    xTaskNotifyGive(self->task); // debería ejecutarse solo en el último segmento+"
    ),
    (
        "static void sdo_download_segment_request(request_t* self) \n{\n    response_t resp = { .run = sdo_download_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .size = self->size };",
        "static void sdo_download_segment_request(request_t* self) \n{\n    response_t resp = { .run = sdo_download_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .size = self->size, .task = self->task };"
    ),
    (
        "        memcpy(req_payload->seg_data, self->value, n);\n        request_t req = { .run = sdo_download_segment_request, .msg = &req_msg, .value = self->value + n, .size = self->size - n };\n        xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    }\n    else { // confirmation of last segment\n        xSemaphoreGive(sdo_sem);\n    }",
        "        memcpy(req_payload->seg_data, self->value, n);\n        request_t req = { .run = sdo_download_segment_request, .msg = &req_msg, .value = self->value + n, .size = self->size - n, .task = self->task };\n        xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    }\n    else { // confirmation of last segment\n        xTaskNotifyGive(self->task);\n    }"
    ),
    (
        "    SDO_download_req_t* payload = (SDO_download_req_t*) self->msg->data;\n    if (payload->e) { // expedited\n        response_t resp = { .run =  sdo_download_response, .cobid = self->msg->identifier - 0x600 + 0x580 };\n        xQueueSend(rx_task_queue, &resp, portMAX_DELAY);\n    }\n    else {\n        response_t resp = { .run =  sdo_download_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .size = self->size };\n        xQueueSend(rx_task_queue, &resp, portMAX_DELAY);\n    }",
        "    SDO_download_req_t* payload = (SDO_download_req_t*) self->msg->data;\n    if (payload->e) { // expedited\n        response_t resp = { .run =  sdo_download_response, .cobid = self->msg->identifier - 0x600 + 0x580, .task = self->task };\n        xQueueSend(rx_task_queue, &resp, portMAX_DELAY);\n    }\n    else {\n        response_t resp = { .run =  sdo_download_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .size = self->size, .task = self->task };\n        xQueueSend(rx_task_queue, &resp, portMAX_DELAY);\n    }"
    ),
    (
        "    request_t req = { .run = sdo_download_request, .msg = &msg, .value = value };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    if (xSemaphoreTake(sdo_sem, maxDelay) != pdTRUE) {",
        "    request_t req = { .run = sdo_download_request, .msg = &msg, .value = value, .task = xTaskGetCurrentTaskHandle() };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    if (ulTaskNotifyTake(pdTRUE, maxDelay) == 0) {"
    ),
    (
        "    if (payload->c) {\n        xSemaphoreGive(sdo_sem);\n    }\n    else {\n        static twai_message_t req_msg;\n        req_msg = (twai_message_t) { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };\n        SDO_upload_seq_req_t* req_payload = (SDO_upload_seq_req_t*) req_msg.data;\n        *req_payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = !payload->t, .reserved = {0} };\n        request_t req = { .run = sdo_upload_segment_request, .msg = &req_msg, .value = self->value + n };\n        xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    }",
        "    if (payload->c) {\n        xTaskNotifyGive(self->task);\n    }\n    else {\n        static twai_message_t req_msg;\n        req_msg = (twai_message_t) { .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };\n        SDO_upload_seq_req_t* req_payload = (SDO_upload_seq_req_t*) req_msg.data;\n        *req_payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = !payload->t, .reserved = {0} };\n        request_t req = { .run = sdo_upload_segment_request, .msg = &req_msg, .value = self->value + n, .task = self->task };\n        xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    }"
    ),
    (
        "static void sdo_upload_segment_request(request_t* self) \n{\n    response_t resp = { .run = sdo_upload_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value };",
        "static void sdo_upload_segment_request(request_t* self) \n{\n    response_t resp = { .run = sdo_upload_segment_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .task = self->task };"
    ),
    (
        "    if (payload->e) {\n        xSemaphoreGive(sdo_sem);\n    }\n    else {\n        static twai_message_t req_msg;\n        req_msg = (twai_message_t){ .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };\n        SDO_upload_seq_req_t* payload = (SDO_upload_seq_req_t*) req_msg.data;\n        *payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = 0, .reserved = {0} };\n        request_t req = { .run = sdo_upload_segment_request, .msg = &req_msg, .value =self->value + n };\n        xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    }",
        "    if (payload->e) {\n        xTaskNotifyGive(self->task);\n    }\n    else {\n        static twai_message_t req_msg;\n        req_msg = (twai_message_t){ .extd = 0, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .identifier = msg->identifier - 0x580 + 0x600, .data_length_code = 8 };\n        SDO_upload_seq_req_t* payload = (SDO_upload_seq_req_t*) req_msg.data;\n        *payload = (SDO_upload_seq_req_t){ .x = 0, .ccs = SDO_CCS_UPLOAD_SEG, .t = 0, .reserved = {0} };\n        request_t req = { .run = sdo_upload_segment_request, .msg = &req_msg, .value =self->value + n, .task = self->task };\n        xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    }"
    ),
    (
        "static void sdo_upload_request(request_t* self) \n{\n    response_t resp = { .run = sdo_upload_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value };",
        "static void sdo_upload_request(request_t* self) \n{\n    response_t resp = { .run = sdo_upload_response, .cobid = self->msg->identifier - 0x600 + 0x580, .value = self->value, .task = self->task };"
    ),
    (
        "    request_t req = { .run = sdo_upload_request, .msg = &msg, .value = ret };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    if (xSemaphoreTake(sdo_sem, maxDelay) != pdTRUE) {",
        "    request_t req = { .run = sdo_upload_request, .msg = &msg, .value = ret, .task = xTaskGetCurrentTaskHandle() };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    if (ulTaskNotifyTake(pdTRUE, maxDelay) == 0) {"
    ),
    (
        "static void nmt_request(request_t* self) \n{\n    dump_msg(\"Transmit\", self->msg);\n    twai_transmit(self->msg, portMAX_DELAY);\n    xSemaphoreGive(nmt_sem);\n}",
        "static void nmt_request(request_t* self) \n{\n    dump_msg(\"Transmit\", self->msg);\n    twai_transmit(self->msg, portMAX_DELAY);\n    xTaskNotifyGive(self->task);\n}"
    ),
    (
        "    request_t req = { .run = nmt_request, .msg = &msg };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    if (xSemaphoreTake(nmt_sem, maxDelay) != pdTRUE) {",
        "    request_t req = { .run = nmt_request, .msg = &msg, .task = xTaskGetCurrentTaskHandle() };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n    if (ulTaskNotifyTake(pdTRUE, maxDelay) == 0) {"
    ),
    (
        "void epos_done()\n{\n    request_t req = { .run = &epos_req_done_run };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n\n    response_t resp = { .run = &epos_resp_done_run };\n    xQueueSend(rx_task_queue, &resp, portMAX_DELAY);\n\n    xSemaphoreGive(done_sem);\n    vTaskDelete(NULL); // exit calling task\n}",
        "void epos_done()\n{\n    request_t req = { .run = &epos_req_done_run };\n    xQueueSend(tx_task_queue, &req, portMAX_DELAY);\n\n    response_t resp = { .run = &epos_resp_done_run };\n    xQueueSend(rx_task_queue, &resp, portMAX_DELAY);\n\n    if (done_task_handle) xTaskNotifyGive(done_task_handle);\n    vTaskDelete(NULL); // exit calling task\n}"
    ),
    (
        "    tx_task_queue = xQueueCreate(1, sizeof(request_t));\n    rx_task_queue = xQueueCreate(1, sizeof(response_t));\n \n    sdo_sem = xSemaphoreCreateBinary();\n    nmt_sem = xSemaphoreCreateBinary();\n    done_sem = xSemaphoreCreateBinary();\n\n    xTaskCreatePinnedToCore(epos_receive_task, \"receive\",   4096, NULL,  8, NULL, tskNO_AFFINITY);",
        "    tx_task_queue = xQueueCreate(1, sizeof(request_t));\n    rx_task_queue = xQueueCreate(1, sizeof(response_t));\n \n    xTaskCreatePinnedToCore(epos_receive_task, \"receive\",   4096, NULL,  8, NULL, tskNO_AFFINITY);"
    ),
    (
        "esp_err_t epos_wait_done()\n{\n    xSemaphoreTake(done_sem, portMAX_DELAY);\n\n    ESP_RETURN_ON_ERROR(twai_stop(), TAG, \"Unable to stop TWAI\");\n    ESP_RETURN_ON_ERROR(twai_driver_uninstall(), TAG, \"Unable to uninstall TWAI driver\");\n\n    vQueueDelete(rx_task_queue);\n    vQueueDelete(tx_task_queue);\n    vSemaphoreDelete(sdo_sem);\n    vSemaphoreDelete(nmt_sem);\n    vSemaphoreDelete(done_sem);\n    return ESP_OK;\n}",
        "esp_err_t epos_wait_done()\n{\n    done_task_handle = xTaskGetCurrentTaskHandle();\n    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);\n\n    ESP_RETURN_ON_ERROR(twai_stop(), TAG, \"Unable to stop TWAI\");\n    ESP_RETURN_ON_ERROR(twai_driver_uninstall(), TAG, \"Unable to uninstall TWAI driver\");\n\n    vQueueDelete(rx_task_queue);\n    vQueueDelete(tx_task_queue);\n    return ESP_OK;\n}"
    ),
    (
        "typedef struct {\n    uint32_t cobid;\n    SemaphoreHandle_t semaphore;\n    bool in_use;\n} cobid_entry_t;",
        "typedef struct {\n    uint32_t cobid;\n    TaskHandle_t task;\n    bool in_use;\n} cobid_entry_t;"
    ),
    (
        "            if (ret) memcpy(ret, data, 8); // copy CANopen payload\n            xSemaphoreGive(cobid_table[i].semaphore);\n            cobid_table[i].in_use = false;",
        "            if (ret) memcpy(ret, data, 8); // copy CANopen payload\n            xTaskNotifyGive(cobid_table[i].task);\n            cobid_table[i].in_use = false;"
    ),
    (
        "    for (int i = 0; i < N_ELEMS(cobid_table); ++i) {\n        if (!cobid_table[i].in_use) {\n            if (cobid_table[i].semaphore == NULL) {\n                cobid_table[i].semaphore = xSemaphoreCreateBinary();\n                if (cobid_table[i].semaphore == NULL) {\n                    ESP_LOGE(TAG, \"Failed to create semaphore for COB-ID %08x\", (unsigned)cobid);\n                    return ESP_ERR_NO_MEM;\n                }\n            }\n            cobid_table[i].cobid = cobid;\n            cobid_table[i].in_use = true;\n\n            epos_register_canopen_handler(cobid, epos_signal_cobid, ret);\n            BaseType_t done = xSemaphoreTake(cobid_table[i].semaphore, maxDelay);\n            epos_unregister_canopen_handler(cobid);\n\n            if (done) return ESP_OK;\n            ESP_LOGE(TAG, \"Timeout waiting for COB-ID %08x\", (unsigned)cobid);\n            return ESP_ERR_TIMEOUT;\n        }\n    }",
        "    for (int i = 0; i < N_ELEMS(cobid_table); ++i) {\n        if (!cobid_table[i].in_use) {\n            cobid_table[i].cobid = cobid;\n            cobid_table[i].task = xTaskGetCurrentTaskHandle();\n            cobid_table[i].in_use = true;\n\n            epos_register_canopen_handler(cobid, epos_signal_cobid, ret);\n            uint32_t done = ulTaskNotifyTake(pdTRUE, maxDelay);\n            epos_unregister_canopen_handler(cobid);\n\n            if (done > 0) return ESP_OK;\n            ESP_LOGE(TAG, \"Timeout waiting for COB-ID %08x\", (unsigned)cobid);\n            return ESP_ERR_TIMEOUT;\n        }\n    }"
    )
]

for idx, (old, new) in enumerate(replacements):
    if old not in content:
        print(f"Error: Could not find chunk {idx} in {filepath}")
        print("OLD STRING:")
        print(old)
        sys.exit(1)
    content = content.replace(old, new, 1)

with open(filepath, "w", encoding="utf-8") as f:
    f.write(content)

print(f"Successfully applied {len(replacements)} replacements to {filepath}")
