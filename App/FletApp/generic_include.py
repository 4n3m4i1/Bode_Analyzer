FL_CVT_CONST = 32768.0

def Q15_to_float(a):
    return float(a / FL_CVT_CONST)

def Q15_to_float_array(a, LENGTH):
    return [Q15_to_float(i) for i in range(LENGTH)]