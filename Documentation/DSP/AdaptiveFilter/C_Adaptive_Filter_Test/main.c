#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <inttypes.h>
#include <math.h>

#define LIMIT_FOR_ERROR     1.0

#define DFL_ITERATIONS      256

#define DFL_BETA_RATE       0.005


uint32_t    print_options = 0;
#define CSV_OI4V_FMT    1u

uint32_t    num_taps = 0;
uint32_t    iterations = DFL_ITERATIONS;

// Different algorithms for H_HAT updates, leave all commented for DFL
//#define HALG_2        // DOESN'T WORK!!!!
//#define HALG_3        // DOESN'T WORK EVEN W /0 FIX!!


// Sorta kinda Box-Muller + Lazy Programming
double generate_awgn_sample(){
    double U1 = drand48();

    return sqrt(-2 * log(U1)) - 1.25;
}


// Simulate a DUT response HERE
double perform_fir(double *ideal_taps, double *idt_delay_line, double new_value, uint32_t len){
    double retval = 0;
    if(ideal_taps && idt_delay_line){
        idt_delay_line[0] = new_value;
        for(uint32_t n = 0; n < len; ++n) retval += ideal_taps[n] * idt_delay_line[n];
        for(uint32_t n = len - 1; n > 0; --n) idt_delay_line[n] = idt_delay_line[n - 1];
    }
    return retval;
}


void update_h_hat(double *h_hat_taps, double *idt_delay_line, double error, double beta_rate, uint32_t len){
    if(h_hat_taps && idt_delay_line){
#ifdef HALG_2
        for(uint32_t n = 0; n < len - 1; ++n){
            h_hat_taps[n + 1] = h_hat_taps[n] + beta_rate * (error * idt_delay_line[n]);
        }
#elif defined(HALG_3)
// Normalized
        for(uint32_t n = 0; n < len; ++n){
            if(!idt_delay_line[n]) idt_delay_line[n] += 1e-12;
            h_hat_taps[n] += (beta_rate * error) / (idt_delay_line[n]);
            printf("HE %lf\n",h_hat_taps[n]);
        }

#else
        for(uint32_t n = 0; n < len; ++n){
            h_hat_taps[n] += beta_rate * (error * idt_delay_line[n]);
        }
#endif
    }
}


void main(int argc, char **argv){
    if(argc > 1){
        iterations = strtoul(argv[1],NULL,10);
    }
    printf("Running %u samples of white noise through the system!\n", iterations);

    double beta = DFL_BETA_RATE;

    if(argc > 2){
        beta = strtod(argv[2],NULL);
    }

    printf("Beta adaptive rate: %lf\n",beta);


    FILE *printfp;
    if(argc > 3){
        print_options = CSV_OI4V_FMT;
        printf("Print History as Images: On!\n");
        printfp = fopen("adaptive_tap_frames.csv","w");
        fprintf(printfp,"%lu\n%.14lf\n", iterations, beta);
    }


    FILE *tapfp;
    tapfp = fopen("test_ideal_tap_weights.txt","r");

    double rval;

    while(fscanf(tapfp,"%lf",&rval) != EOF) num_taps += 1;
    printf("Number of Ideal Taps: %u\n",num_taps);
    rewind(tapfp);

    double *ideal_taps = malloc(num_taps * sizeof(double));
    double *ideal_delay_line = malloc(num_taps * sizeof(double));

    double *adaptive_taps = malloc(num_taps * sizeof(double));
    double *x = malloc(num_taps * sizeof(double));

    double err;

    double *error_plot = malloc(iterations * sizeof(double));

    double error_accum = 0;

    if(ideal_taps && adaptive_taps && x && ideal_delay_line && error_plot){
        
        for(uint32_t n = 0; n < num_taps; ++n){
            fscanf(tapfp,"%lf",&rval);
            ideal_taps[n] = rval;
        
            adaptive_taps[n] = 0;
            x[n] = 0;
            ideal_delay_line[n] = 0;
        }

        // Save ideal taps
        if(print_options == CSV_OI4V_FMT){
            fprintf(printfp,"%lu\n",num_taps);          // Print number of taps
            for(uint32_t q = 0; q < num_taps - 1; ++q){
                fprintf(printfp,"%.14lf,",ideal_taps[q]);
            }
            fprintf(printfp,"%.14lf\n",ideal_taps[num_taps - 1]);
        }
        
        // Run main sampling loop simulation
        for(uint32_t n = 0; n < iterations; ++n){
            double new_data = generate_awgn_sample();
            
            // Simulate a system under test's output given white noise input
            double target_sample_d = perform_fir(ideal_taps, ideal_delay_line, new_data, num_taps);

            // Run "internal" FIR structure
            double y_hat = perform_fir(adaptive_taps, x, new_data, num_taps);
        
            // Determine output error
            err = target_sample_d - y_hat;
            
            // For statistics
            error_accum += err;
            error_plot[n] = err;


            // Iterate and reconfigure adaptive taps to correct for measured
            //  error.
            update_h_hat(adaptive_taps, x, err, beta, num_taps);
        

            // Add new line for newest tap stuff
            if(print_options == CSV_OI4V_FMT){
                for(uint32_t q = 0; q < num_taps - 1; ++q){
                    fprintf(printfp,"%.14lf,",adaptive_taps[q]);
                }
                fprintf(printfp,"%.14lf\n",adaptive_taps[num_taps - 1]);
            }
        }

        // Close file, turn into video
        if(print_options == CSV_OI4V_FMT){
            fclose(printfp);
        }

        double error_avg = fabs(error_accum / (double)iterations);

        if(error_avg > LIMIT_FOR_ERROR){
            printf("Failed to converge under error limit!! :^(\n");
        } else {
            printf("Completed simulation with an average error of:\n\t%.14lf\n",error_avg);    
        }
        


        // Setup output for:
        //      Ideal Taps
        //      h_hat
        //      e
        //      Tap error

        if(error_avg <= LIMIT_FOR_ERROR){
            FILE *fpo;
            fpo = fopen("simulation_results.csv","w");
            fprintf(fpo,"IDX,H_HAT,H_IDEAL,H_ERROR\n");
            for(uint32_t n = 0; n < num_taps; ++n){
                fprintf(fpo,"%u,%.14lf,%.14lf,%.14lf\n", n, adaptive_taps[n], ideal_taps[n], adaptive_taps[n] - ideal_taps[n]);
            }
            fclose(fpo);

            fpo = fopen("error_plot.csv","w");
            fprintf(fpo,"IDX,ERR\n");
            for(uint32_t n = 0; n < iterations; ++n){
                fprintf(fpo,"%u,%.14lf\n",n,error_plot[n]);
            }
            fclose(fpo);
        }
        

        free(ideal_taps);
        free(adaptive_taps);
        free(x);
        free(ideal_delay_line);
        free(error_plot);
    } else {
        printf("Error allocating memory!\n");
    }
    fclose(tapfp);
}