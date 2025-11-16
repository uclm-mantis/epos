#ifndef MOVING_AVERAGE
#define MOVING_AVERAGE

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

/*! @brief Macro para generar la declaración de un objeto de media móvil

    Define el tipo y declara las funciones para cálculo de media móvil y varianza móvil para
    un tamaño y un tipo dado.  Se pasa el literal que inicializa la suma @a sum0 y extrae
    el tipo de la suma y varianza de este literal. Esto permite emplear @a float o @a double
    para acumular las muestras enteras cuando la profundidad es grande.

    @see MOVING_AVERAGE_IMPL
    @param base_type Tipo de las muestras.
    @param size      Tamaño de la media móvil (en muestras).
    @param sum0      Literal correspondiente a cero para la suma.
*/
#define MOVING_AVERAGE_TYPE(base_type, size, sum0) \
    typedef struct { size_t head; base_type samples[size]; __typeof__(sum0) sum; __typeof__(sum0) sum2; } MovingAverage_##size##_##base_type; \
    void moving_average_##size##_##base_type##_init(MovingAverage_##size##_##base_type* self); \
    base_type moving_average_##size##_##base_type##_push(MovingAverage_##size##_##base_type* self, base_type v); \
    base_type moving_average_##size##_##base_type##_stddev(MovingAverage_##size##_##base_type* self);

/*! @brief Macro para generar la implementación de un objeto de media móvil

    Define las funciones para cálculo de media móvil y varianza móvil para un tamaño y un 
    tipo dado.  Se pasa el literal que inicializa la suma @a sum0 y extrae el tipo de la 
    suma y varianza de este literal. Esto permite emplear @a float o @a double
    para acumular las muestras enteras cuando la profundidad es grande.

    @see MOVING_AVERAGE_DECL
    @param base_type Tipo de las muestras.
    @param size      Tamaño de la media móvil (en muestras).
    @param sum0      Literal correspondiente a cero para la suma.
*/
#define MOVING_AVERAGE_IMPL(base_type, size, sum0) \
void moving_average_##size##_##base_type##_init(MovingAverage_##size##_##base_type* self) { \
    self->head = 0; self->sum = sum0; memset(self->samples, 0, sizeof(self->samples)); \
} \
base_type moving_average_##size##_##base_type##_push(MovingAverage_##size##_##base_type* self, base_type v) { \
    base_type old = self->samples[self->head]; \
    self->sum += v - old; self->sum2 += v*v - old*old; \
    self->samples[self->head++] = v; self->head %= size ; \
    return self->sum / size; \
} \
base_type moving_average_##size##_##base_type##_var(MovingAverage_##size##_##base_type* self) { \
    return (self->sum2 - self->sum * self->sum) / size; \
}


MOVING_AVERAGE_TYPE(int16_t, 16, 0L)

#ifdef __cplusplus
}
#endif

#endif
