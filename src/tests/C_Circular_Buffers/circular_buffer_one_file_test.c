#include <stdio.h>
#include <inttypes.h>
#include <malloc.h>


struct Circular_Buffer_Int32 {
    uint16_t    size;
    uint16_t    size_mask;      // for 2^n len
    uint16_t    curr_zero;
    int32_t     *data;
};

void setup_circular_buffer(struct Circular_Buffer_Int32 *a, int size){
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

void static inline add_sample_to_buffer(struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero] = val;
    if(++a->curr_zero >= a->size) a->curr_zero = 0;
}

void static inline add_sample_to_2n_buffer_I32(const struct Circular_Buffer_Int32 *a, int32_t val){
    a->data[a->curr_zero++ & a->size_mask] = val;
}

int32_t static inline recall_sample_from_buffer(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(((offset + a->curr_zero) >= a->size) ? a->curr_zero + (offset -= a->size) : (a->curr_zero + offset))];
}

int32_t static inline recall_sample_from_2n_buffer(struct Circular_Buffer_Int32 *a, int offset){
    return a->data[(offset + a->curr_zero) & a->size_mask];
}

void destroy_circular_buffer(struct Circular_Buffer_Int32 *a){
    if(a){
        free(a->data);
    }
}

void main(){
    struct Circular_Buffer_Int32 sbuf;

    int buflen = 64;

    setup_circular_buffer(&sbuf, buflen);

    for(int n = 0; n < buflen; ++n){
        sbuf.data[n] = n;
    }

    // Simulae FIR style fill
    for(int m = 0; m < 12; ++m){        // 12 samples go in
        //add_sample_to_buffer(&sbuf, m + 100);
        add_sample_to_2n_buffer_I32(&sbuf, m+100);
    }

    sbuf.curr_zero = 0;

    for(int n = 0; n < buflen << 1; ++n){
        printf("Buffer[%3d] = %d\n", n % buflen, recall_sample_from_2n_buffer(&sbuf, n));
    }

    destroy_circular_buffer(&sbuf);
}