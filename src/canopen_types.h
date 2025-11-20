#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
    NOTA (FMF): Intento utilizar la misma nomenclatura que el estándar CANopen CiA 301 y la misma descomposición
    en protocolos independientes. 
 */

enum {
    SDO_CCS_DOWNLOAD_SEG,       // download segment request
    SDO_CCS_DOWNLOAD,           // initiate download request
    SDO_CCS_UPLOAD,             // initiate upload request
    SDO_CCS_UPLOAD_SEG,         // upload segment request
    SDO_CCS_UPLOAD_BLK = 5,     // upload block request
    SDO_CCS_DOWNLOAD_BLK,       // download block request
};

enum {
    SDO_SCS_UPLOAD_SEG,         // upload segment response
    SDO_SCS_DOWNLOAD_SEG,       // download segment response
    SDO_SCS_UPLOAD,             // initiate upload response
    SDO_SCS_DOWNLOAD,           // initiate download response
    SDO_SCS_DOWNLOAD_BLK = 5,   // block download
    SDO_SCS_UPLOAD_BLK,         // block upload
};


// 7.2.4.3.2 Protocol SDO download ===================================================================================

// 7.2.4.3.3 Protocol SDO download initiate ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t s: 1;   // en download 1 = se indica tamaño, 0 = no se indica tamaño
            uint8_t e: 1;   // en download 1:expedita, 0: segmentada, en upload se ignora
            uint8_t n: 2;   // bytes del payload que no contienen datos (valido si e = 1 y s = 1), 0 en caso contrario
            uint8_t x : 1;  // ignorado, debe ser 0
            uint8_t ccs: 3; // client command specifier
        };
        uint8_t command;
    };
    union {
        struct __attribute__((packed)) {
            uint16_t index;
            uint8_t subindex;
        };
        uint8_t m[3];
    };
    union {
        uint8_t d[4];
        uint32_t dsize;
    };
} SDO_download_req_t;

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t x : 5;  // ignorado, debe ser 0
            uint8_t scs: 3; // server command specifier
        };
        uint8_t command;
    };
    union {
        struct __attribute__((packed)) {
            uint16_t index;
            uint8_t subindex;
        };
        uint8_t m[3];
    };
    uint8_t reserved[4];    // debe ser 0
} SDO_download_resp_t;

// 7.2.4.3.4 Protocol SDO download segment ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t c: 1;   // 0 = hay mas segmentos, 1 = ultimo segmento
            uint8_t n: 3;   // bytes del payload que no contienen datos
            uint8_t t : 1;  // toggle bit, debe conmutar entre segmentos
            uint8_t ccs: 3; // client command specifier
        };
        uint8_t command;
    };
    uint8_t seg_data[7];
} SDO_download_seg_req_t;

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t x : 4;  // ignorado, debe ser 0
            uint8_t t : 1;  // igual que en la petición
            uint8_t scs: 3; // server command specifier
        };
        uint8_t command;
    };
    uint8_t reserved[7];    // debe ser 0
} SDO_download_seg_resp_t;



// 7.2.4.3.5 Protocol SDO upload ====================================================================================

// 7.2.4.3.6 Protocol SDO upload initiate ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t x : 5;  // ignorado, debe ser 0
            uint8_t ccs: 3; // client command specifier
        };
        uint8_t command;
    };
    union {
        struct __attribute__((packed)) {
            uint16_t index;
            uint8_t subindex;
        };
        uint8_t m[3];
    };
    uint8_t reserved[4];    // debe ser 0
} SDO_upload_req_t;

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t s: 1;   // en download 1 = se indica tamaño, 0 = no se indica tamaño
            uint8_t e: 1;   // en download 1:expedita, 0: segmentada, en upload se ignora
            uint8_t n: 2;   // bytes del payload que no contienen datos (valido si e = 1 y s = 1), 0 en caso contrario
            uint8_t x : 1;  // ignorado, debe ser 0
            uint8_t scs: 3; // server command specifier
        };
        uint8_t command;
    };
    union {
        struct __attribute__((packed)) {
            uint16_t index;
            uint8_t subindex;
        };
        uint8_t m[3];
    };
    uint8_t d[4];
} SDO_upload_resp_t;


// 7.2.4.3.7 Protocol SDO upload segment ------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t x : 4;  // ignorado, debe ser 0
            uint8_t t : 1;  // igual que en la petición
            uint8_t ccs: 3; // client command specifier
        };
        uint8_t command;
    };
    uint8_t reserved[7];    // debe ser 0
} SDO_upload_seq_req_t;

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t c: 1;   // 0 = hay mas segmentos, 1 = ultimo segmento
            uint8_t n: 3;   // bytes del payload que no contienen datos
            uint8_t t : 1;  // toggle bit, debe conmutar entre segmentos
            uint8_t scs: 3; // server command specifier
        };
        uint8_t command;
    };
    uint8_t seg_data[7];
} SDO_upload_seq_resp_t;

// FIXME: FMF: De momento no implementaremos el SDO block download/upload porque creo que Maxon tampoco lo hace


// 7.2.8 Network management ============================================================================================

enum {
    NMT_START_REMOTE_NODE = 0x01,
    NMT_STOP_REMOTE_NODE,
    NMT_ENTER_PREOPERATIONAL = 0x80,
    NMT_RESET_NODE,
    NMT_RESET_COMMUNICATION,
};

typedef struct __attribute__((packed)) {
    uint8_t cs;     // control service
    uint8_t nodeID; // 0 = broadcast, normalmente n
} NMT_t;


/*
    AS es una macro que puede usarse con estructuras empaquetadas para construir 
    valores numéricos.

    AS(uint16_t, ((ControlWord_PPM_t){.fault_reset = 1}))
*/
#define AS(T,v) ((union { T data; typeof(v) value; }){.value = v}).data

#ifdef __cplusplus
}
#endif
