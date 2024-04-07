#include "RP2040_Circ_Buffer.h"

#ifdef USE_CIRC_BUFFER

#ifdef ALLOW_32_SIZE_BUFFER
void setup_circular_buffer_I32(struct Circular_Buffer_Int32 *a, int size){
    if(a){
        a->size = size;
        a->size_mask = 0;
        a->curr_zero = 0;
        a->data = malloc(size * sizeof(int32_t));

        int ones_ct = 0;
        for(int n = 0; n < 32; ++n){
            if(a->size & (1u << n)) ones_ct++;
        }

        if(ones_ct == 1){
            int n;
            for(n = 0; n < 32; ++n){
                if((a->size >> n) == 1) break;
                else {
                    a->size_mask |= (1u << n);
                }
            }
        }
    }
}

#ifdef ALLOW_NON_2N_BUFFER
inline   void add_sample_to_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}
#endif

inline   void add_sample_to_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

#ifdef ALLOW_NON_2N_BUFFER
inline   int32_t recall_sample_from_buffer_I32(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}
#endif

inline   int32_t recall_sample_from_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

#ifdef ALLOW_NON_2N_BUFFER
void destroy_circular_buffer_I32(struct Circular_Buffer_Int32 *a){
    if(a){
        free(a->data);
    }
}
#endif

#endif


void setup_circular_buffer_I16(struct Circular_Buffer_Int16 *a, int size){
    if(a){
        a->size = size;
        a->size_mask = 0;
        a->curr_zero = 0;
        a->data = malloc(size * sizeof(int16_t));

        int ones_ct = 0;
        for(int n = 0; n < 16; ++n){
            if(a->size & (1u << n)) ones_ct++;
        }

        if(ones_ct == 1){
            int n;
            for(n = 0; n < 16; ++n){
                if((a->size >> n) == 1) break;
                else {
                    a->size_mask |= (1u << n);
                }
            }
        }
    }
}

#ifdef ALLOW_NON_2N_BUFFER
inline   void add_sample_to_buffer_I16(struct Circular_Buffer_Int16 *a, int16_t val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}
#endif

inline   void add_sample_to_2n_buffer_I16(struct Circular_Buffer_Int16 *a, int16_t val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

#ifdef ALLOW_NON_2N_BUFFER
inline   int16_t recall_sample_from_buffer_I16(struct Circular_Buffer_Int16 *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}
#endif

inline   int16_t recall_sample_from_2n_buffer_I16(struct Circular_Buffer_Int16 *a, uint16_t offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

#ifdef ALLOW_NON_2N_BUFFER
void destroy_circular_buffer_I16(struct Circular_Buffer_Int16 *a){
    if(a){
        free(a->data);
    }
}

#endif
#endif