# Multirate Downsampling
Reduction of cost and space necessitates moving a multirate sampling solution into the digital domain.  
This example will show the process, pitfalls, and solutions with implementing such a mechanism.  
Specifically the simulation of a fixed high Fs ADC with a dynamic downsampler in the digital domain.  
In the context of this project this will allow variable rate FFT analysis and higher resolutions at lower  
frequency ranges without sacrificing analytical abilities at higher frequencies of interest.
