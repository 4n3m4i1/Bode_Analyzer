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
  
This results in:
$`V_{out}=\frac{1}{2}V_Scos\left(\phi \right)`$
Given this result we run into a small issue, at certain phase offsets we lose the ability to properly resolve the signal.  
This is due to the constant term going to zero at a $`\phi =\pi `$ phase offset.  
To fix this issue we can simply multiply our input by two signals: the aforementioned sine term, and a cosine term.  
In this way we observe two results:  
$`X=\frac{V_S}{2}cos\left(\phi \right)`$  
$`Y=\frac{V_S}{2}sin\left(\phi \right)`$  
In this way we create the vector components representing the amplitude and phase of our input signal relative to the reference cosine.  
To recover the phase offset of the wave, which much always be a delay to maintain causality (the only outcome of real world circuits) as well as reference  
the input to the generated sine wave we see: $`\theta =\frac{\pi }{2}-tan^{-1}\left(\frac{Y}{X}\right)`$ gives us the correct result.  
To recover amplitude we utilize the simple vector magnitude equation: $`V_S\:=\:\frac{2}{V_R}\cdot \sqrt{X^2+Y^2}`$  
  
A simple Octave script is provided detailing the operation of this method.  
With a simple block diagram it can be seen the raw maths can be accomplished within a small FPGA using a minimal amount of hardware multipliers.  
This fits within the use case of the Lattice UP series, with 2 multipliers allocated towards signal multiplication, and all others allocated  
towards low pass filtering the output of this multiply. This averaged signal is what creates both the $`X`$ and $`Y`$ signals.  
  
Unfortunately the square root and atan() operations are far too expensive to run in such a small FPGA, as such external means must be implemented  
to accomplish these tasks (external MCU or computer).
