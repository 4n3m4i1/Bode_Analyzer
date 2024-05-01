from enum import IntEnum

FL_CVT_CONST = 32768.0
# FL_CVT_CONST = 1
def Q15_to_float(a):
    return float(a / FL_CVT_CONST)

def Q15_to_float_array(a, LENGTH):
    return [Q15_to_float(a[i]) for i in range(LENGTH)]

class Port():
    def __init__(self, portString = None):
        self.name = portString
    def set(self, port):
        self.name = port

class BANDIT_SETTINGS_BYTES(IntEnum):
    USBBSRX_EN = 0
    USBBSRX_AUTORUN = 1
    USBBSRX_AUTOSEND = 2
    USBBSRX_WGN_ALWAYS_ON = 3
    USBBSRX_RESERVED = 4
    USBBSRX_ERR_LSB = 5
    USBBSRX_ERR_MSB = 6
    USBBSRX_TAPLEN_LSB = 7
    USBBSRX_TAPLEN_MSB = 8
    USBBSRX_F_FRANGE = 9
    USBBSRX_OFFSET_LSB = 10
    USBBSRX_OFFSET_MSB = 11
    USBBSRX_ATTEMPTS_LSB = 12
    USBBSRX_ATTEMPTS_MSB = 13
    USBBSRX_LEARNING_RATE_LSB = 14
    USBBSRX_LEARNING_RATE_MSB = 15
    USBBSRX_RAW_RQ = 16
    USBBSRX_TIME_DOMAIN_DATA = 17
    USBBSRX_SINGLE_SHOT = 18
    USBBSRX_CORE_1_DBG_RQ = 19
class FRANGE_ENUM(IntEnum):
    FRANGE_250K = 0
    FRANGE_125K = 1
    FRANGE_62K5 = 2
    FRANGE_31K25 = 3
FRANGE_VAL = [250e3, 125e3, 62.5e3, 31.25e3]
SETTINGS_BF_LEN = 20
INIT_SETTINGS = [
    0x1,  #ENABLE
    0x1,  #AUTORUN
    0x1,  #AUTOSEND
    0x1,  #WGN ALWAYS ON
    0x00, #RESERVED
    0x20, #ERR LSB
    0x00, #ERR MSB
    0x40, #TAPLEN LSB
    0x00, #TAPLEN MSB
    0x00, #FRANGE ENUM
    0x00, #OFFSET LSB
    0x00, #OFFSET MSB
    0x04, #ATTEMPTS LSB
    0x00, #ATTEMPTS MSB
    0x33, #LEARNING RATE LSB
    0x03, #LEARNING RATE MSB
    0x00, #RAW RQ
    0x00, #TIME DOMAIN DATA RQ
    0x00, #SINGLE SHOT
    0x00  #DEBGUG RQ
    ]
init_graph = [((_ * _) + 1e-15) for _ in range(128)]

MAX_TAPS = 1024

MAX_QUEUE_SIZE = 3

MIN_QUEUE_SIZE = 1

parametric_taps =[32,64,128,256, 512, 1024]

frequency_ranges = ["250K","125K","62.5K", "32.25K"]