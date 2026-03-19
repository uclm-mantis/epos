#include "canopen_client.h"

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_GETTER(r,id,sid,t)
#include "object_dictionary.h"
#undef OBJ

#define OBJ(id,sid,d,i,t,rp,tp,r,w) GENERATE_SETTER(w,id,sid,t)
#include "object_dictionary.h"
#undef OBJ
