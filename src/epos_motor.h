#pragma once

#include "canopen_console_types.h"
#include "canopen_client.h"

#define OBJ(id,sid,d,i,t,rp,tp,r,w) CANOPEN_CLIENT_DECLARE_GETTER(r,id,sid,t)
#include "client_od.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) CANOPEN_CLIENT_DECLARE_SETTER(w,id,sid,t)
#include "client_od.h"
#undef OBJ

enum {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) CANOPEN_CLIENT_DECLARE_TX_MAPPING_ENUM(i,tp,id,sid,t)
#include "client_od.h"
#undef OBJ
};

enum {
#define OBJ(id,sid,d,i,t,rp,tp,r,w) CANOPEN_CLIENT_DECLARE_RX_MAPPING_ENUM(i,rp,id,sid,t)
#include "client_od.h"
#undef OBJ
};
