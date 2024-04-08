from enum import Enum

FL_CVT_CONST = 32768.0

def Q15_to_float(a):
    return float(a / FL_CVT_CONST)

def Q15_to_float_array(a, LENGTH):
    return [Q15_to_float(a[i]) for i in range(LENGTH)]

class BANDIT_SETTINGS_BYTES(Enum):
    BS_EN = 0
    BS_AUTORUN = 1
    BS_AUTOSEND = 2
    BS_WGN_ALWAYS_ON = 3
    BS_RESERVED = 4
    BS_ERR_LSB = 5
    BS_EFF_MSB = 6
    BS_TAPLEN_LSB = 7
    BS_TAPLEN_MSB = 8
    BS_F_FRANGE = 9
SETTINGS_BF_LEN = 10
INIT_SETTINGS = [0x1, 0x1, 0x1, 0x01, 0x00, 0x66, 0x06, 0x20, 0x00, 0x00]