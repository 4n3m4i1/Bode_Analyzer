import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, fft
import struct
from itertools import chain
from queue import Queue

def serial_read(dataPortName, ctrlPortName, data_Queue: Queue):
    BYTES_PER_NUMBER = 2
    CDC_PACKET_LENGTH = 64
    DATA_PACKET_LENGTH = 128
    HEADER_PACKET_LENGTH = 64
    H = 1
    HH = 2
    FFR = 3
    FFI = 4
    IDLE = 5   
    STATE = 5
    DATACHANNEL = serial.Serial(
        port = dataPortName,
        baudrate = 9600,
        bytesize = serial.EIGHTBITS,
        timeout = 1,

    )
    CTRLCHANNEL = serial.Serial(
        port= ctrlPortName,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        timeout=1,
    )
    # x = 0

    while True:

        # DATACHANNEL.reset_input_buffer()
        match STATE:
            case 1:
                CTRLCHANNEL.write(b'a')
                HEADER = DATACHANNEL.read(HEADER_PACKET_LENGTH)

                # print(HEADER.hex(' '))
                # print('\n')
                STATE = HH
            case 2:
                CTRLCHANNEL.write(b'a')
                HHat = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)

                HHat = struct.iter_unpack('d', HHat)
                HHat = list(chain.from_iterable(HHat))
                # print(HHat)
                # print('\n')
                STATE = FFR
            case 3:
                CTRLCHANNEL.write(b'a')
                FFR_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                data_Queue.put(FFR_data)
                print(FFR_data.hex(' '))
                print('\n')
                STATE = FFI
            case 4:
                CTRLCHANNEL.write(b'a')
                FFI_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                # print(FFI_data.hex(' '))
                # print('\n')
                STATE = IDLE
            case 5:
                CTRLCHANNEL.write(b'a')
                IDLE_data = DATACHANNEL.read(CDC_PACKET_LENGTH * BYTES_PER_NUMBER)
                # print(IDLE_data.hex(' '))
                # print('\n')
                # y_fft = fft.fft(HHat)

                # y_fft = np.abs(y_fft)
                # y_fft = fft.fftshift(y_fft)

                # plt.plot(y_fft)
                # plt.show()
                # break
                STATE = H




if __name__ == "__main__":
     
    serial_read("/dev/tty.usbmodem1234561", "/dev/tty.usbmodem1234563")

                