#include "canopen_client.h"

#define OBJ(id,sid,d,i,t,rp,tp,r,w) CANOPEN_CLIENT_DEFINE_GETTER(r,id,sid,t)
#include "client_od.h"

#define OBJ(id,sid,d,i,t,rp,tp,r,w) CANOPEN_CLIENT_DEFINE_SETTER(w,id,sid,t)
#include "client_od.h"
