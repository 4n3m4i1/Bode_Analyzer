# DSP Theory
The principles of lock in amplifiers are utilized for noise resistant amplitude and phase  
detection. These principles are simple, easy to parallelize, and are listed below.  
  
A sinusoidal signal:
$`V_s\left(t\right)=V_Xsin\left(\omega _st\:+\:\phi \right)`$  
is provided to a system.  
This system can multiply this input by a sine wave $`V_R\left(t\right)=V_Rsin\left(\omega _Rt\right)`$  
This results in the output $`V_R\left(t\right)V_S\left(t\right)=\frac{1}{2}V_RV_X\left[cos\left[\left(\omega _St\:+\:\omega _Rt\right)\:+\:\phi \right]\:-\:cos\left[\left(\omega \:_{St}\:-\:\omega \:_{Rt}\right)\:+\:\phi \:\right]\right]`$  
This may look long, however given out reference frequency is equal to our signal frequency, or: $`\omega _S=\omega _R`$  
A lot of this cancels over time.
