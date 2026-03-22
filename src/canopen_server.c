#include "canopen_server.h"

#include <string.h>
#include "esp_check.h"
#include "esp_log.h"

#define N_ELEMS(x) (sizeof(x) / sizeof((x)[0]))
#define MAX_CANOPEN_SDO_SERVERS 8
#define CANOPEN_SERVER_TRANSFER_MAX 256

static const char *TAG = "CANOPEN_SERVER";

/* -------------------------------------------------------------------------- */
/*  Wrappers generados desde server_od.h                                      */
/* -------------------------------------------------------------------------- */

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_GETTER_WRAPPER(r, t)
#include "server_od.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_SETTER_WRAPPER(w, t)
#include "server_od.h"
#undef OBJ

/* -------------------------------------------------------------------------- */
/*  Tabla OD servidor                                                         */
/* -------------------------------------------------------------------------- */

static const canopen_server_od_entry_t server_od[] = {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_OD_ENTRY(id, sid, t, r, w),
#include "server_od.h"
#undef OBJ
};

/* -------------------------------------------------------------------------- */
/*  Estado por canal SDO servidor                                             */
/* -------------------------------------------------------------------------- */

typedef enum {
    CANOPEN_SERVER_SDO_IDLE = 0,
    CANOPEN_SERVER_SDO_DOWNLOAD_SEG,
    CANOPEN_SERVER_SDO_UPLOAD_SEG,
} canopen_server_sdo_state_t;

typedef struct {
    bool in_use;
    uint8_t node_id;
    uint32_t cobid_req;
    uint32_t cobid_resp;
    canopen_handler_handle_t handle;

    canopen_server_sdo_state_t state;
    uint16_t index;
    uint8_t subindex;
    uint8_t toggle;

    uint8_t buf[CANOPEN_SERVER_TRANSFER_MAX];
    size_t size;
    size_t offset;
} canopen_server_instance_t;

static canopen_server_instance_t server_instances[MAX_CANOPEN_SDO_SERVERS];

/* -------------------------------------------------------------------------- */
/*  Abort codes                                                               */
/* -------------------------------------------------------------------------- */

uint32_t canopen_od_status_to_abort_code(canopen_od_status_t st)
{
    switch (st) {
    case CANOPEN_OD_OK:                 return 0x00000000u;
    case CANOPEN_OD_UNSUPPORTED_ACCESS: return 0x06010000u;
    case CANOPEN_OD_WRITEONLY:          return 0x06010001u;
    case CANOPEN_OD_READONLY:           return 0x06010002u;
    case CANOPEN_OD_NO_SUCH_OBJECT:     return 0x06020000u;
    case CANOPEN_OD_NO_SUCH_SUBINDEX:   return 0x06090011u;
    case CANOPEN_OD_TYPE_MISMATCH:      return 0x06070010u;
    case CANOPEN_OD_DATA_TOO_LONG:      return 0x06070012u;
    case CANOPEN_OD_DATA_TOO_SHORT:     return 0x06070013u;
    case CANOPEN_OD_INVALID_VALUE:      return 0x06090030u;
    case CANOPEN_OD_HW_ERROR:           return 0x06060000u;
    default:                            return 0x08000000u;
    }
}

/* -------------------------------------------------------------------------- */
/*  Helpers internos                                                          */
/* -------------------------------------------------------------------------- */

static void server_reset_transfer(canopen_server_instance_t *inst)
{
    inst->state = CANOPEN_SERVER_SDO_IDLE;
    inst->index = 0;
    inst->subindex = 0;
    inst->toggle = 0;
    inst->size = 0;
    inst->offset = 0;
}

static const canopen_server_od_entry_t *
canopen_server_find_internal(uint16_t index, uint8_t subindex, bool *same_index_found)
{
    if (same_index_found) {
        *same_index_found = false;
    }

    for (size_t i = 0; i < N_ELEMS(server_od); ++i) {
        if (server_od[i].index == index) {
            if (same_index_found) {
                *same_index_found = true;
            }
            if (server_od[i].subindex == subindex) {
                return &server_od[i];
            }
        }
    }
    return NULL;
}

const canopen_server_od_entry_t *canopen_server_find(uint16_t index, uint8_t subindex)
{
    return canopen_server_find_internal(index, subindex, NULL);
}

static canopen_od_status_t canopen_server_entry_get(const canopen_server_od_entry_t *entry,
                                                    void *value,
                                                    size_t size)
{
    if (entry == NULL) {
        return CANOPEN_OD_NO_SUCH_OBJECT;
    }
    if (entry->get == NULL) {
        return CANOPEN_OD_WRITEONLY;
    }
    if (size < entry->size) {
        return CANOPEN_OD_DATA_TOO_SHORT;
    }

    memset(value, 0, size);
    return entry->get(value);
}

static canopen_od_status_t canopen_server_entry_set(const canopen_server_od_entry_t *entry,
                                                    const void *value,
                                                    size_t size)
{
    if (entry == NULL) {
        return CANOPEN_OD_NO_SUCH_OBJECT;
    }
    if (entry->set == NULL) {
        return CANOPEN_OD_READONLY;
    }
    if (size > entry->size) {
        return CANOPEN_OD_DATA_TOO_LONG;
    }
    if (size < entry->size) {
        return CANOPEN_OD_DATA_TOO_SHORT;
    }

    return entry->set(value);
}

canopen_od_status_t canopen_server_od_get(uint16_t index,
                                          uint8_t subindex,
                                          void *value,
                                          size_t size)
{
    bool same_index_found = false;
    const canopen_server_od_entry_t *entry =
        canopen_server_find_internal(index, subindex, &same_index_found);

    if (entry == NULL) {
        return same_index_found ? CANOPEN_OD_NO_SUCH_SUBINDEX
                                : CANOPEN_OD_NO_SUCH_OBJECT;
    }
    return canopen_server_entry_get(entry, value, size);
}

canopen_od_status_t canopen_server_od_set(uint16_t index,
                                          uint8_t subindex,
                                          const void *value,
                                          size_t size)
{
    bool same_index_found = false;
    const canopen_server_od_entry_t *entry =
        canopen_server_find_internal(index, subindex, &same_index_found);

    if (entry == NULL) {
        return same_index_found ? CANOPEN_OD_NO_SUCH_SUBINDEX
                                : CANOPEN_OD_NO_SUCH_OBJECT;
    }
    return canopen_server_entry_set(entry, value, size);
}

static void sdo_make_abort(uint16_t index,
                           uint8_t subindex,
                           uint32_t abort_code,
                           void *resp8)
{
    uint8_t *resp = (uint8_t *)resp8;
    memset(resp, 0, 8);
    resp[0] = 0x80u;
    memcpy(&resp[1], &index, sizeof(index));
    resp[3] = subindex;
    memcpy(&resp[4], &abort_code, sizeof(abort_code));
}

static esp_err_t canopen_server_send(uint32_t cobid, const void *payload8)
{
    twai_message_t msg = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = cobid,
        .data_length_code = 8,
    };

    memcpy(msg.data, payload8, 8);
    return canopen_post(&msg); // solo encola, no espera
}

/* -------------------------------------------------------------------------- */
/*  Subprotocolos servidor                                                    */
/* -------------------------------------------------------------------------- */

static esp_err_t sdo_server_download(canopen_server_instance_t *inst,
                                     const SDO_download_req_t *req,
                                     uint8_t resp8[8])
{
    bool same_index_found = false;
    const canopen_server_od_entry_t *entry =
        canopen_server_find_internal(req->index, req->subindex, &same_index_found);

    SDO_download_resp_t *resp = (SDO_download_resp_t *)resp8;
    memset(resp8, 0, 8);

    if (entry == NULL) {
        sdo_make_abort(req->index, req->subindex,
                       canopen_od_status_to_abort_code(
                           same_index_found ? CANOPEN_OD_NO_SUCH_SUBINDEX
                                            : CANOPEN_OD_NO_SUCH_OBJECT),
                       resp8);
        return ESP_OK;
    }

    if (entry->set == NULL) {
        sdo_make_abort(req->index, req->subindex,
                       canopen_od_status_to_abort_code(CANOPEN_OD_READONLY),
                       resp8);
        return ESP_OK;
    }

    server_reset_transfer(inst);

    if (req->e) {
        size_t size = req->s ? (size_t)(4u - req->n) : entry->size;
        canopen_od_status_t st = canopen_server_od_set(req->index, req->subindex, req->d, size);
        if (st != CANOPEN_OD_OK) {
            sdo_make_abort(req->index, req->subindex,
                           canopen_od_status_to_abort_code(st),
                           resp8);
            return ESP_OK;
        }

        *resp = (SDO_download_resp_t){
            .scs = SDO_SCS_DOWNLOAD,
            .index = req->index,
            .subindex = req->subindex,
        };
        return ESP_OK;
    }

    if (!req->s) {
        sdo_make_abort(req->index, req->subindex, 0x06070010u, resp8);
        return ESP_OK;
    }

    if (req->dsize > sizeof(inst->buf)) {
        sdo_make_abort(req->index, req->subindex,
                       canopen_od_status_to_abort_code(CANOPEN_OD_DATA_TOO_LONG),
                       resp8);
        return ESP_OK;
    }

    inst->state = CANOPEN_SERVER_SDO_DOWNLOAD_SEG;
    inst->index = req->index;
    inst->subindex = req->subindex;
    inst->toggle = 0;
    inst->size = req->dsize;
    inst->offset = 0;

    *resp = (SDO_download_resp_t){
        .scs = SDO_SCS_DOWNLOAD,
        .index = req->index,
        .subindex = req->subindex,
    };
    return ESP_OK;
}

static esp_err_t sdo_server_download_segment(canopen_server_instance_t *inst,
                                             const SDO_download_seg_req_t *req,
                                             uint8_t resp8[8])
{
    SDO_download_seg_resp_t *resp = (SDO_download_seg_resp_t *)resp8;
    memset(resp8, 0, 8);

    if (inst->state != CANOPEN_SERVER_SDO_DOWNLOAD_SEG) {
        sdo_make_abort(inst->index, inst->subindex, 0x05040001u, resp8);
        return ESP_OK;
    }

    if (req->t != inst->toggle) {
        sdo_make_abort(inst->index, inst->subindex, 0x05030000u, resp8);
        server_reset_transfer(inst);
        return ESP_OK;
    }

    size_t nbytes = 7u - req->n;
    if (inst->offset + nbytes > inst->size || inst->offset + nbytes > sizeof(inst->buf)) {
        sdo_make_abort(inst->index, inst->subindex,
                       canopen_od_status_to_abort_code(CANOPEN_OD_DATA_TOO_LONG),
                       resp8);
        server_reset_transfer(inst);
        return ESP_OK;
    }

    memcpy(&inst->buf[inst->offset], req->seg_data, nbytes);
    inst->offset += nbytes;

    *resp = (SDO_download_seg_resp_t){
        .scs = SDO_SCS_DOWNLOAD_SEG,
        .t = req->t,
    };

    if (req->c) {
        if (inst->offset != inst->size) {
            sdo_make_abort(inst->index, inst->subindex,
                           canopen_od_status_to_abort_code(CANOPEN_OD_DATA_TOO_SHORT),
                           resp8);
            server_reset_transfer(inst);
            return ESP_OK;
        }

        canopen_od_status_t st = canopen_server_od_set(inst->index,
                                                       inst->subindex,
                                                       inst->buf,
                                                       inst->size);
        if (st != CANOPEN_OD_OK) {
            sdo_make_abort(inst->index, inst->subindex,
                           canopen_od_status_to_abort_code(st),
                           resp8);
            server_reset_transfer(inst);
            return ESP_OK;
        }

        server_reset_transfer(inst);
        return ESP_OK;
    }

    inst->toggle ^= 1u;
    return ESP_OK;
}

static esp_err_t sdo_server_upload(canopen_server_instance_t *inst,
                                   const SDO_upload_req_t *req,
                                   uint8_t resp8[8])
{
    bool same_index_found = false;
    const canopen_server_od_entry_t *entry =
        canopen_server_find_internal(req->index, req->subindex, &same_index_found);

    SDO_upload_resp_t *resp = (SDO_upload_resp_t *)resp8;
    memset(resp8, 0, 8);

    if (entry == NULL) {
        sdo_make_abort(req->index, req->subindex,
                       canopen_od_status_to_abort_code(
                           same_index_found ? CANOPEN_OD_NO_SUCH_SUBINDEX
                                            : CANOPEN_OD_NO_SUCH_OBJECT),
                       resp8);
        return ESP_OK;
    }

    if (entry->get == NULL) {
        sdo_make_abort(req->index, req->subindex,
                       canopen_od_status_to_abort_code(CANOPEN_OD_WRITEONLY),
                       resp8);
        return ESP_OK;
    }

    server_reset_transfer(inst);

    canopen_od_status_t st = canopen_server_od_get(req->index,
                                                   req->subindex,
                                                   inst->buf,
                                                   sizeof(inst->buf));
    if (st != CANOPEN_OD_OK) {
        sdo_make_abort(req->index, req->subindex,
                       canopen_od_status_to_abort_code(st),
                       resp8);
        return ESP_OK;
    }

    inst->size = entry->size;
    inst->offset = 0;
    inst->index = req->index;
    inst->subindex = req->subindex;
    inst->toggle = 0;

    if (entry->size <= 4) {
        *resp = (SDO_upload_resp_t){
            .scs = SDO_SCS_UPLOAD,
            .s = 1,
            .e = 1,
            .n = (uint8_t)(4u - entry->size),
            .index = req->index,
            .subindex = req->subindex,
        };
        memcpy(resp->d, inst->buf, entry->size);
        server_reset_transfer(inst);
        return ESP_OK;
    }

    *resp = (SDO_upload_resp_t) {
        .scs = SDO_SCS_UPLOAD,
        .s = 1,
        .e = 0,
        .n = 0,
        .index = req->index,
        .subindex = req->subindex,
    };
    memcpy(resp->d, &inst->size, sizeof(uint32_t));
    inst->state = CANOPEN_SERVER_SDO_UPLOAD_SEG;
    return ESP_OK;
}

static esp_err_t sdo_server_upload_segment(canopen_server_instance_t *inst,
                                           const SDO_upload_seq_req_t *req,
                                           uint8_t resp8[8])
{
    SDO_upload_seq_resp_t *resp = (SDO_upload_seq_resp_t *)resp8;
    memset(resp8, 0, 8);

    if (inst->state != CANOPEN_SERVER_SDO_UPLOAD_SEG) {
        sdo_make_abort(inst->index, inst->subindex, 0x05040001u, resp8);
        return ESP_OK;
    }

    if (req->t != inst->toggle) {
        sdo_make_abort(inst->index, inst->subindex, 0x05030000u, resp8);
        server_reset_transfer(inst);
        return ESP_OK;
    }

    size_t remaining = inst->size - inst->offset;
    size_t nbytes = remaining < 7u ? remaining : 7u;
    bool last = (nbytes == remaining);

    *resp = (SDO_upload_seq_resp_t){
        .scs = SDO_SCS_UPLOAD_SEG,
        .t = req->t,
        .c = last ? 1u : 0u,
        .n = (uint8_t)(7u - nbytes),
    };
    memcpy(resp->seg_data, &inst->buf[inst->offset], nbytes);

    inst->offset += nbytes;

    if (last) {
        server_reset_transfer(inst);
    } else {
        inst->toggle ^= 1u;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  Dispatcher                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t canopen_server_dispatch_sdo(uint32_t cobid_req,
                                      const void *req,
                                      size_t req_size,
                                      uint32_t *cobid_resp,
                                      void *resp,
                                      size_t *resp_size)
{
    uint8_t frame[8] = {0};

    ESP_RETURN_ON_FALSE(req != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL req");
    ESP_RETURN_ON_FALSE(resp != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL resp");
    ESP_RETURN_ON_FALSE(cobid_resp != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL cobid_resp");
    ESP_RETURN_ON_FALSE(resp_size != NULL, ESP_ERR_INVALID_ARG, TAG, "NULL resp_size");
    ESP_RETURN_ON_FALSE(req_size <= 8, ESP_ERR_INVALID_ARG, TAG, "req too large");

    memcpy(frame, req, req_size);
    *cobid_resp = (cobid_req - 0x600u) + 0x580u;
    *resp_size = 8;

    /* Esta función genérica ya no mantiene estado. El estado vive en la instancia. */
    sdo_make_abort(0, 0, 0x08000000u, resp);
    return ESP_ERR_NOT_SUPPORTED;
}

/* -------------------------------------------------------------------------- */
/*  Handler registrado en canopen                                             */
/* -------------------------------------------------------------------------- */

static void canopen_server_sdo_handler(uint32_t cobid, void *data, void *context)
{
    canopen_server_instance_t *inst = (canopen_server_instance_t *)context;
    const uint8_t *frame = (const uint8_t *)data;
    uint8_t resp8[8] = {0};
    esp_err_t err = ESP_OK;

    if (inst == NULL || frame == NULL) {
        return;
    }

    switch ((frame[0] >> 5) & 0x7u) {
    case SDO_CCS_DOWNLOAD:
        err = sdo_server_download(inst, (const SDO_download_req_t *)frame, resp8);
        break;

    case SDO_CCS_DOWNLOAD_SEG:
        err = sdo_server_download_segment(inst, (const SDO_download_seg_req_t *)frame, resp8);
        break;

    case SDO_CCS_UPLOAD:
        err = sdo_server_upload(inst, (const SDO_upload_req_t *)frame, resp8);
        break;

    case SDO_CCS_UPLOAD_SEG:
        err = sdo_server_upload_segment(inst, (const SDO_upload_seq_req_t *)frame, resp8);
        break;

    case SDO_CCS_UPLOAD_BLK:
    case SDO_CCS_DOWNLOAD_BLK:
    default:
        sdo_make_abort(0, 0, 0x05040001u, resp8);
        break;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SDO server handler failed for node %u", (unsigned)inst->node_id);
        return;
    }

    if (canopen_server_send(inst->cobid_resp, resp8) != ESP_OK) {
        ESP_LOGE(TAG, "SDO response transmit failed for node %u", (unsigned)inst->node_id);
    }
}

/* -------------------------------------------------------------------------- */
/*  Lifecycle                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t canopen_server_start(uint8_t node_id)
{
    uint32_t cobid_req = 0x600u + node_id;
    uint32_t cobid_resp = 0x580u + node_id;

    for (size_t i = 0; i < N_ELEMS(server_instances); ++i) {
        if (server_instances[i].in_use && server_instances[i].node_id == node_id) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    for (size_t i = 0; i < N_ELEMS(server_instances); ++i) {
        if (!server_instances[i].in_use) {
            server_instances[i].in_use = true;
            server_instances[i].node_id = node_id;
            server_instances[i].cobid_req = cobid_req;
            server_instances[i].cobid_resp = cobid_resp;
            server_instances[i].handle = NULL;
            server_reset_transfer(&server_instances[i]);

            esp_err_t err = canopen_register_handler(cobid_req,
                                                     canopen_server_sdo_handler,
                                                     &server_instances[i],
                                                     &server_instances[i].handle);
            if (err != ESP_OK) {
                memset(&server_instances[i], 0, sizeof(server_instances[i]));
                return err;
            }

            return ESP_OK;
        }
    }

    return ESP_ERR_NO_MEM;
}

esp_err_t canopen_server_stop(uint8_t node_id)
{
    for (size_t i = 0; i < N_ELEMS(server_instances); ++i) {
        if (server_instances[i].in_use && server_instances[i].node_id == node_id) {
            esp_err_t err = ESP_OK;

            if (server_instances[i].handle != NULL) {
                err = canopen_unregister_handler(server_instances[i].handle);
            }

            memset(&server_instances[i], 0, sizeof(server_instances[i]));
            return err;
        }
    }

    return ESP_ERR_NOT_FOUND;
}