# Bode Bandits Analysis Tool
The Bode Bandits Analysis Tool provides frequency response characteristics of a multitude of filters.
A Python GUI allows interface with the tool, providing configuration and visual representations of the measured transfer function.  
  
## Hardware
Board and assembly files are provided in the repository, optimized for JLC's PCBA service.  
  
## Methods
The Least Mean Squared (LMS) adaptive filter methodology is implemented in a semi-real time fashion, allowing rapid characterization cycles to be made.  
All algorithms (LMS, FFT, Downsampling) are fully parametric, and largely user selectable on the fly. Automatic self-calibration corrects for  
component tolerance variations, VDDA magnitude, and the frontend anti-aliasing filter transfer function. DC calibration routines are run at  
startup, all transfer function calibration metrics are gathered at startup as well as upon change of any parametric setting.  
  
![](https://github.com/4n3m4i1/Bode_Analyzer/blob/main/img/anim_output.gif)
  
## Real Time Adjustment Parameters
- Range of frequency analysis (set ranges)
- Datapoint resolution (2^n)
- Acceptable LMS convergence error tolerances
  
## Required User Software 
- Python 3.9+
- That's it :)
  
## Repo Layout
- App: Contains the PC side of the project. Including the Python source and helper tools.
- HDL: Placeholder for potential HDL in the event a switch to a more PL based approach occurs.
- src: All C source for both the current main build as well as helper C files and feature tests.
- Documentation: All standard documentation as well as theory behind every working component, with test programs and scrips included.
