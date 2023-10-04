#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>


#define DEFAULT_FS      1000000

void main(int argc, char **argv){
    uint32_t Fs = DEFAULT_FS;

    if(argc > 1){
        for(int n = 1; n < argc; ++n){

            uint8_t rchar = 0;            
            char tmp[16] = {0x00};


            if(argv[n][0] == '-' && ((n + 1) < argc)){
                switch(argv[n][1]){
                    case 'F':
                        Fs = strtoul(argv[++n],NULL,10);
                    break;

                    case 'w':

                    break;

                    case '0':

                    break;

                    case '1':

                    break;

                    default:
                        printf("Unrecognized command %c!\n",argv[n][1]);
                    break;
                }
            }
        }
    }
    



}