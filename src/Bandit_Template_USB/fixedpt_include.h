#ifndef GENERIC_INCLUDE_h
#define GENERIC_INCLUDE_h

#include <inttypes.h>
// Macros for Q15 fixed point

#define SHAMT       15  // initially 15
typedef int16_t     Q15;
typedef int32_t     _sn2xFix;
typedef uint32_t    _uns2xFix;
//#define multfix14_16(a,b) ((fix14_16)((((signed long long)(a))*((signed long long)(b)))>>15)) //multiply two fixed 16.15
#define add_Q15(a,b)        (a + b)
#define sub_Q15(a,b)        (a - b)
#define mul_Q15(a,b)        ((Q15)((((_sn2xFix)(a))*((_sn2xFix)(b)))>>SHAMT)) //multiply two fixed 16.15

#define abs_Q15(a)          abs(a)
#define int_2_Q15(a)        ((Q15)(a << SHAMT))
#define Q15_2_int32(a)      ((_sn2xFix)(a >> SHAMT))
#define Q15_2_uint32(a)     ((_uns2xFix)(a >> SHAMT))
#define char_2_Q15(a)       (((Q15)(a)) << SHAMT)


// Don't include to save prog space even tho compiler probably is smart enough
#ifdef FIX_2_FLOAT

#define FL_CVT_CONST        32768.0 // prev 32768.0
#define float_2_Q15(a)      ((Q15)((a)*FL_CVT_CONST)) // 2^SHAMT
#define Q15_2_float(a)      ((float)(a)/FL_CVT_CONST)

#endif


#endif