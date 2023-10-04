#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <inttypes.h>
// Macros for Q15 fixed point

#define SHAMT           15  // initially 15
typedef int16_t Q15;
//#define multfix14_16(a,b) ((fix14_16)((((signed long long)(a))*((signed long long)(b)))>>15)) //multiply two fixed 16.15
#define add_Q15(a,b)        (a + b)
#define sub_Q15(a,b)        (a - b)
#define mul_Q15(a,b)        ((Q15)((((int32_t)(a))*((int32_t)(b)))>>SHAMT)) //multiply two fixed 16.15

#define abs_Q15(a)          abs(a)
#define int_2_Q15(a)        ((Q15)(a << SHAMT))
#define Q15_2_int32(a)      ((int32_t)(a >> SHAMT))
#define Q15_2_uint32(a)     ((uint32_t)(a >> SHAMT))
#define char_2_Q15(a)       (((Q15)(a)) << SHAMT)

#define FL_CVT_CONST        32768.0 // prev 32768.0
#define float_2_Q15(a)      ((Q15)((a)*FL_CVT_CONST)) // 2^SHAMT
#define Q15_2_float(a)      ((float)(a)/FL_CVT_CONST)


#define DFL_NUM_SAMP        1024

void setup_table_file(FILE *fp, const char *table_name, uint32_t sample_ct);

void main(int argc, char **argv){
    uint32_t nsamp = DFL_NUM_SAMP;
    if(argc > 1) nsamp = strtoul(argv[1],NULL,10);

    FILE *sin_ofp;
    sin_ofp = fopen("Q15_sin_table.h","w");

    setup_table_file(sin_ofp,"sin_table",nsamp);
    
    for(uint32_t n = 0; n < nsamp; ++n){
        double sin_term = sin((2.0 * M_PI) * (((double)n)/((double)nsamp)));
        fprintf(sin_ofp,"0x%04X",(uint16_t)float_2_Q15(sin_term - (1 / FL_CVT_CONST)));
        if(n != (nsamp - 1)) fprintf(sin_ofp,",\n\t\t");
    }
    fprintf(sin_ofp, "\n};\n");


    fclose(sin_ofp);

    if(argc > 2){
        if(argv[2][0] == '-' && argv[2][1] == 'c'){
            FILE *cos_ofp;
            cos_ofp = fopen("Q15_cos_table.h","w");

            setup_table_file(cos_ofp,"cos_table",nsamp);
    
            for(uint32_t n = 0; n < nsamp; ++n){
                double cos_term = cos((2.0 * M_PI) * (((double)n)/((double)nsamp)));
                fprintf(cos_ofp,"0x%04X",(uint16_t)float_2_Q15(cos_term - (1 / FL_CVT_CONST)));
                if(n != (nsamp - 1)) fprintf(cos_ofp,",\n\t\t");
            }
            fprintf(cos_ofp, "\n};\n");

            fclose(cos_ofp);
        }
    }    
}


void setup_table_file(FILE *fp, const char *table_name, uint32_t sample_ct){
    fprintf(fp,"/*\nThis table is autogenerated, please do not alter! - Joseph\n*/\n\n");
    fprintf(fp, "Q15 %s_table[%u] = {\n\t\t", table_name, sample_ct);
}
