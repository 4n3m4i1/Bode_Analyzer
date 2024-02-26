#ifndef RP2040_CIRC_BUFFER_h
#define RP2040_CIRC_BUFFER_h

#include "pico/stdlib.h"
#include <stdlib.h>
#include "pico/malloc.h"
#include <inttypes.h>

/*
    Lil circular buffer library, buffers of 2^n are way faster to work with
*/


struct Circular_Buffer_Int32 {
    uint16_t    size;
    uint16_t    size_mask;      // for 2^n len
    uint16_t    curr_zero;
    int32_t     *data;
};

struct Circular_Buffer_Int16 {
    uint16_t    size;
    uint16_t    size_mask;      // for 2^n len
    uint16_t    curr_zero;
    int16_t     *data;
};

void setup_circular_buffer_I32(struct Circular_Buffer_Int32 *a, int size);

void static inline add_sample_to_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val);
void static inline add_sample_to_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val);
int32_t static inline recall_sample_from_buffer_I32(struct Circular_Buffer_Int32 *a, int offset);
int32_t static inline recall_sample_from_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int offset);
void destroy_circular_buffer_I32(struct Circular_Buffer_Int32 *a);


void setup_circular_buffer_I16(struct Circular_Buffer_Int16 *a, int size);

void static inline add_sample_to_buffer_I16(struct Circular_Buffer_Int16 *a, int16_t val);
void static inline add_sample_to_2n_buffer_I16(struct Circular_Buffer_Int16 *a, int32_t val);
int16_t static inline recall_sample_from_buffer_I16(struct Circular_Buffer_Int16 *a, int offset);
int16_t static inline recall_sample_from_2n_buffer_I16(struct Circular_Buffer_Int16 *a, int offset);
void destroy_circular_buffer_I16(struct Circular_Buffer_Int16 *a);

#endif