#ifndef RP2040_CIRC_BUFFER_h
#define RP2040_CIRC_BUFFER_h

#include "pico/stdlib.h"
#include <stdlib.h>
#include "pico/malloc.h"
#include <inttypes.h>

/*
    Lil circular buffer library, buffers of 2^n are way faster to work with
*/

#ifdef USE_CIRC_BUFFER

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


#ifdef ALLOW_32_SIZE_BUFFER
void setup_circular_buffer_I32(struct Circular_Buffer_Int32 *a, int size);

#ifdef ALLOW_NON_2N_BUFFER
inline   void add_sample_to_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val);
#endif

inline   void add_sample_to_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val);

#ifdef ALLOW_NON_2N_BUFFER
inline   int32_t recall_sample_from_buffer_I32(struct Circular_Buffer_Int32 *a, int offset);
#endif

inline   int32_t recall_sample_from_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int offset);

#ifdef ALLOW_NON_2N_BUFFER
void destroy_circular_buffer_I32(struct Circular_Buffer_Int32 *a);
#endif

#endif

void setup_circular_buffer_I16(struct Circular_Buffer_Int16 *a, int size);

#ifdef ALLOW_NON_2N_BUFFER
inline   void add_sample_to_buffer_I16(struct Circular_Buffer_Int16 *a, int16_t val);
#endif

inline   void add_sample_to_2n_buffer_I16(struct Circular_Buffer_Int16 *a, int16_t val);

#ifdef ALLOW_NON_2N_BUFFER
inline   int16_t recall_sample_from_buffer_I16(struct Circular_Buffer_Int16 *a, int offset);
#endif

inline   int16_t recall_sample_from_2n_buffer_I16(struct Circular_Buffer_Int16 *a, uint16_t offset);

#ifdef ALLOW_NON_2N_BUFFER
void destroy_circular_buffer_I16(struct Circular_Buffer_Int16 *a);
#endif

#endif

#endif