#include <stdio.h>
#include <inttypes.h>

#define LONGFIX

#ifdef LONGFIX
typedef int32_t FT;
typedef int64_t FT_L;
#else
typedef int16_t FT;
typedef int32_t FT_L;
#endif


void main(){
    printf("Start\n");
    FILE *infp;
    infp = fopen("debug_2.log", "r");

    FILE *outfp;
    outfp = fopen("int_debug.log", "w");

    printf("Files Opened!\n");

    char rbuff[256];

    int64_t averager = 0;
    int count = 0;

#ifdef LONGFIX
    FT max_int = INT32_MIN;
    FT min_int = INT32_MAX;
#else
    FT max_int = INT16_MIN;
    FT min_int = INT16_MAX;
#endif

    while(fgets(rbuff, sizeof(rbuff), infp)){
        int n = 0;
        while(rbuff[n++] != ' ');
        
        uint8_t rval[16];

        sscanf(&rbuff[n], "%02X %02X %02X %02X %02X %02X %02X %02X   %02X %02X %02X %02X %02X %02X %02X %02X",
                &rval[0],
                &rval[1], 
                &rval[2], 
                &rval[3], 
                &rval[4],
                &rval[5], 
                &rval[6], 
                &rval[7], 
                &rval[8], 
                &rval[9], 
                &rval[10], 
                &rval[11], 
                &rval[12], 
                &rval[13], 
                &rval[14], 
                &rval[15]
                );

#ifdef LONGFIX
        FT vals[4];
        vals[0] = (int16_t)(rval[0] | (rval[1] << 8) | (rval[2] << 16) | (rval[3] << 24));
        vals[1] = (int16_t)(rval[4] | (rval[5] << 8) | (rval[6] << 16) | (rval[7] << 24));
        vals[2] = (int16_t)(rval[8] | (rval[9] << 8) | (rval[10] << 16) | (rval[11] << 24));
        vals[3] = (int16_t)(rval[12] | (rval[13] << 8) | (rval[14] << 16) | (rval[15] << 24));
#else
        FT vals[8];
        vals[0] = (int16_t)(rval[0] | (rval[1] << 8));
        vals[1] = (int16_t)(rval[2] | (rval[3] << 8));
        vals[2] = (int16_t)(rval[4] | (rval[5] << 8));
        vals[3] = (int16_t)(rval[6] | (rval[7] << 8));
        vals[4] = (int16_t)(rval[8] | (rval[9] << 8));
        vals[5] = (int16_t)(rval[10] | (rval[11] << 8));
        vals[6] = (int16_t)(rval[12] | (rval[13] << 8));
        vals[7] = (int16_t)(rval[14] | (rval[15] << 8));
#endif

#ifdef LONGFIX
        for(int n = 0; n < 4; ++n){
            averager += vals[n];
            if(vals[n] > max_int) max_int = vals[n];
            if(vals[n] < min_int) min_int = vals[n];
        }

        fprintf(outfp, "%d\n%d\n%d\n%d\n",
                vals[0],
                vals[1],
                vals[2],
                vals[3]               
                );
        
        count += 4;
#else
        for(int n = 0; n < 8; ++n){
            averager += vals[n];
            if(vals[n] > max_int) max_int = vals[n];
            if(vals[n] < min_int) min_int = vals[n];
        }
        
        fprintf(outfp, "%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n",
                vals[0],
                vals[1],
                vals[2],
                vals[3],
                vals[4],
                vals[5],
                vals[6],
                vals[7]                
                );
        
        count += 8;
#endif
        
    }

    averager /= count;
    printf("Average:\t%d\n", averager);
    printf("Samples:\t%d\n", count);

    printf("Max:\t\t%d\n", max_int);
    printf("Min:\t\t%d\n", min_int);

    fclose(infp);
    fclose(outfp);

    printf("Done!\n");
}