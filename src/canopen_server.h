#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
    server_od.h debe contener entradas del estilo:

    OBJ(0x2230, 0x1, "Gear Ratio Numerator", gear_ratio_numerator,
        uint32_t, 0, 0, get_gear_ratio_numerator, set_gear_ratio_numerator)

    y puede usar NA en getter y/o setter.
*/

/* ========================================================================== */
/*  Estado / abort codes del OD servidor                                      */
/* ========================================================================== */

typedef enum {
    CANOPEN_OD_OK = 0,
    CANOPEN_OD_UNSUPPORTED_ACCESS,
    CANOPEN_OD_WRITEONLY,
    CANOPEN_OD_READONLY,
    CANOPEN_OD_NO_SUCH_OBJECT,
    CANOPEN_OD_NO_SUCH_SUBINDEX,
    CANOPEN_OD_TYPE_MISMATCH,
    CANOPEN_OD_INVALID_VALUE,
    CANOPEN_OD_DATA_TOO_LONG,
    CANOPEN_OD_DATA_TOO_SHORT,
    CANOPEN_OD_HW_ERROR,
} canopen_od_status_t;

/*
    Conversión a abort code SDO (CiA 301).
    La implementación irá en canopen_server.c.
*/
uint32_t canopen_od_status_to_abort_code(canopen_od_status_t st);

/* ========================================================================== */
/*  Signaturas uniformes para el dispatcher                                   */
/* ========================================================================== */

typedef canopen_od_status_t (*canopen_server_get_fn_t)(void *value);
typedef canopen_od_status_t (*canopen_server_set_fn_t)(const void *value);

typedef struct {
    uint16_t index;
    uint8_t subindex;
    size_t size;
    canopen_server_get_fn_t get;
    canopen_server_set_fn_t set;
} canopen_server_od_entry_t;

/* ========================================================================== */
/*  Declaraciones tipadas de callbacks del usuario                            */
/* ========================================================================== */

#define GENERATE_SERVER_GETTER_DECL(x, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (canopen_od_status_t on_##x(T *value);)

#define GENERATE_SERVER_SETTER_DECL(x, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (canopen_od_status_t on_##x(T value);)

/*
    Genera las declaraciones de:
        on_get_foo(T *value);
        on_set_foo(T value);
*/
#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_GETTER_DECL(r, t)
#include "server_od.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_SETTER_DECL(w, t)
#include "server_od.h"
#undef OBJ

/* ========================================================================== */
/*  Wrappers uniformes para la tabla                                          */
/* ========================================================================== */

#define GENERATE_SERVER_GETTER_WRAPPER(x, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (static inline canopen_od_status_t wrap_##x(void *value) { \
        return on_##x((T *)value); \
    })

#define GENERATE_SERVER_SETTER_WRAPPER(x, T) \
    IF_ELSE(IS_NA(x)) \
    ( ) \
    (static inline canopen_od_status_t wrap_##x(const void *value) { \
        return on_##x(*(const T *)value); \
    })

/* ========================================================================== */
/*  Constructores de tabla                                                    */
/* ========================================================================== */

#define SERVER_GETTER_FN(x) \
    IF_ELSE(IS_NA(x)) \
    ( NULL ) \
    ( &wrap_##x )

#define SERVER_SETTER_FN(x) \
    IF_ELSE(IS_NA(x)) \
    ( NULL ) \
    ( &wrap_##x )

#define GENERATE_SERVER_OD_ENTRY(idx, subidx, T, r, w) \
    { \
        .index = (idx), \
        .subindex = (subidx), \
        .size = sizeof(T), \
        .get = SERVER_GETTER_FN(r), \
        .set = SERVER_SETTER_FN(w), \
    }

/*
    Para usar en canopen_server.c:

    #define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_GETTER_WRAPPER(r,t)
    #include "server_od.h"

    #define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_SETTER_WRAPPER(w,t)
    #include "server_od.h"

    static const canopen_server_od_entry_t server_od[] = {
    #define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SERVER_OD_ENTRY(id,sid,t,r,w),
    #include "server_od.h"
    };
*/

/* ========================================================================== */
/*  API del servidor SDO                                                      */
/* ========================================================================== */

/*
    Registra el/los handlers SDO servidor para un nodo dado.
    La implementación puede usar los COB-ID por defecto:
        rx = 0x600 + node_id
        tx = 0x580 + node_id
*/
esp_err_t canopen_server_start(uint8_t node_id);

/*
    Desregistra handlers y detiene el servidor SDO para ese nodo.
*/
esp_err_t canopen_server_stop(uint8_t node_id);

/*
    Dispatcher explícito para una petición SDO recibida.
    Útil si más adelante quieres desacoplar el registro del handler CAN.
*/
esp_err_t canopen_server_dispatch_sdo(uint32_t cobid_req,
                                      const void *req,
                                      size_t req_size,
                                      uint32_t *cobid_resp,
                                      void *resp,
                                      size_t *resp_size);

/*
    Acceso directo al OD servidor, útil también fuera del protocolo SDO.
*/
canopen_od_status_t canopen_server_od_get(uint16_t index,
                                          uint8_t subindex,
                                          void *value,
                                          size_t size);

canopen_od_status_t canopen_server_od_set(uint16_t index,
                                          uint8_t subindex,
                                          const void *value,
                                          size_t size);

/*
    Búsqueda en la tabla generada.
*/
const canopen_server_od_entry_t *canopen_server_find(uint16_t index,
                                                     uint8_t subindex);

#ifdef __cplusplus
}
#endif