## Joseph A. De Vico
##  10-8-2023
##  Massive CSV -> PNG -> Movie... horrific efficiency, but pretty :)
##

import sys
import time
import matplotlib.pyplot as plt
import numpy as np


def main(argv):
    print("\nFile Okay!");
    dat_file = open(argv[0],"r")
    # Store num pts as first entry
    file_len = int(dat_file.readline())
    print(f"\t     Iterations: {file_len}")
    num_chars = len(str(file_len))

    # Store Beta
    learning_rate = float(dat_file.readline())
    print(f"\t  Learning Rate: {learning_rate}")

    # Get frame stride to reduce file sizes
    frame_stride = int(dat_file.readline())
    print(f"\t   Frame Stride: {frame_stride}")

    # Get number of adaptive taps
    adaptive_tap_ct = int(dat_file.readline())
    print(f"Number of Adaptive Taps: {adaptive_tap_ct}\n")

    ideal_tap_ct = int(dat_file.readline())
    print(f"   Number of Ideal Taps: {ideal_tap_ct}\n")

    print(" Starting Image Generation")

    t0 = time.time()
    adaptive_Xax = np.linspace(0,adaptive_tap_ct - 1,adaptive_tap_ct)
    ideal_Xax = np.linspace(0,adaptive_tap_ct - 1,ideal_tap_ct)
    ##print(adaptive_Xax)

    plt.rcParams["font.family"] = "monospace"

    # First Line == Ideal Taps
    ideal_taps = dat_file.readline().split(',')
    ideal_taps = [float(q) for q in ideal_taps]
    #print(ideal_taps)
    
    number_of_pics = 0

    # Then: IDX vs. H_HATs
    ctr = 0
    ctr_char_w = 1
    shamt = num_chars - ctr_char_w
    for n in dat_file:
        spl = n.split(',')
        fly = [float(q) for q in spl]
        #print(fly)
        # now we finally have an array of floats....
        coolstr = "iter_frame_" + str(number_of_pics) + ".png"
        
        plt.plot(ideal_Xax,ideal_taps, color='b',label='ideal')
        
        frame_label_ctr = str(ctr) + " "*shamt
        plt.plot(adaptive_Xax,fly, color='r', label='adaptive ' + frame_label_ctr + '/' + str(file_len))
        
        plt.xlabel("Tap") 
        plt.ylabel("Magnitude") 
        plt.title(str(adaptive_tap_ct) + " Tap Adaptive Filter @ " + str(learning_rate) + " Alpha") 

        plt.legend()

        plt.savefig('./oi4v/' + coolstr, bbox_inches='tight')
        #plt.show()
        plt.clf()
        number_of_pics = number_of_pics + 1
        ctr = ctr + frame_stride
        ctr_char_w = len(str(ctr))
        shamt = num_chars - ctr_char_w

    dat_file.close()
    t1 = time.time()
    t1 = (t1 - t0)
    print(f"Completed Image Generation in: \t{t1} s")
    print(f"Generated {number_of_pics} images!\n")



if __name__ == "__main__":
    if(len(sys.argv) > 1): 
        main(sys.argv[1:])

    exit()

