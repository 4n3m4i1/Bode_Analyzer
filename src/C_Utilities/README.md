# C Utilities
Small tools to generate tables, convert numbers, etc..  
  
## Signal Table Generator
(Unfinished)  
  
## Q15 Sin Table Generator
Generates Q15 format sin() and cos() tables of arbitrary length.  
Output is in standard C header format w/ hex data representation, and can be directly included in a project, `fixedpt_include.h` is required for type handling.

Default behavior will generate only a sin() table.

Arguments:
- `LLLLL` First argument parameter is length in samples desired
- `-c` if following the length argument will generate a cos() table as well  
  
Future arguments:
- `-b` maximum data bits, to preempt bit growth issues if they eclipse precision in terms of importance  
  
## Float 2 Q15 Converter
Converts a single input float/doubl value, or list of newline seperated float/doubles into Q15 format. Accepts range of [1 - (1/2^15) , -1], otherwise truncation will occur.  
  
Default behavior is: `./converter {value}` and will product a report containing the conversion results, alongside measured error with reference to double precision.  
  
File handling not yet finished.
