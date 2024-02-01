#include "RP2040_Circ_Buffer.h"

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


void static inline add_sample_to_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}

void static inline add_sample_to_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

int32_t static inline recall_sample_from_buffer_I32(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}

int32_t static inline recall_sample_from_2n_buffer_I32(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

void destroy_circular_buffer_I32(struct Circular_Buffer_Int32 *a){
    if(a){
        free(a->data);
    }
}




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

void static inline add_sample_to_buffer_I16(struct Circular_Buffer_Int16 *a, int16_t val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}

void static inline add_sample_to_2n_buffer_I16(struct Circular_Buffer_Int16 *a, int32_t val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

int16_t static inline recall_sample_from_buffer_I16(struct Circular_Buffer_Int16 *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}

int16_t static inline recall_sample_from_2n_buffer_I16(struct Circular_Buffer_Int16 *a, int offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

void destroy_circular_buffer_I16(struct Circular_Buffer_Int16 *a){
    if(a){
        free(a->data);
    }
}