from enum import Enum

FL_CVT_CONST = 32768.0
# FL_CVT_CONST = 1
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
    BS_RAW_RQ = 10
    BS_TD = 11
    BS_SINGLE_SHOT = 12
    BS_DEBUG_RQ = 13
class FRANGE_ENUM(Enum):
    FRANGE_250K = 0
    FRANGE_125K = 1
    FRANGE_62K5 = 2
    FRANGE_31K25 = 3
FRANGE_VAL = [250e3, 125e3, 62.5e3, 31.25e3]
SETTINGS_BF_LEN = 10
INIT_SETTINGS = [0x1, 0x1, 0x1, 0x0, 0x00, 0x66, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
init_graph = [1e-15 for _ in range(128)]