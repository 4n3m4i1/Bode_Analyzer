#include <stdio.h>
#include <stdlib.h>
#define FIX_2_FLOAT
#include "../fixedpt_include.h"



void print_val_as_Q15(double in);

char *filepath;
uint8_t readfile = 0;


void main(int argc, char **argv){
    if(argc == 2){  // Calc input val
        double ab = strtod(argv[1],NULL);
        print_val_as_Q15(ab);
    } else
    if(argc > 2){
        if(argv[1][0] == '-'){
            switch(argv[1][1]){
                case 'P':
                    filepath = argv[2];
                    printf("Reading file: %s\n", filepath);
                    readfile = 1;
                break;
            }
        }
        printf("Not implemented yet :)\n");
    }



    if(readfile){
        double rval;
        FILE *fp = fopen(filepath, "r");
        if(fp){
            while(fscanf(fp, "%lf\n", &rval) != EOF){
                
            }    
        }
        
    }


}



void print_val_as_Q15(double in){
    printf("Entered:      %.14lf\n", in);
    Q15 aa = float_2_Q15(in);
    ///char tmp[8] = {0x00};
    //sprintf(tmp,"%04X",aa);
    printf("Float -> Q15: 0x%04X\n", aa);
    float bcv = Q15_2_float(aa);
    printf("Q15 -> Float: %.14lf\n", bcv);
    printf("Abs Error:    %.14lf\n", in - bcv);
    printf("%% Error:      %.4lf%%\n", 100 * ((bcv - in) / in));

}