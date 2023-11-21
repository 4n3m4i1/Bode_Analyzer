import serial
CDC_PACKET_LENGTH = 64
DATA_PACKET_LENGTH = 128
START = b"0"

def serial_read(dataPortName, ctrlPortName):
    STATE = -1
    DATACHANNEL = serial.Serial(
        port = dataPortName,
        baudrate = 9600,
        bytesize = 8,
        timeout = 1,

    )
    CTRLCHANNEL = serial.Serial(
        port= ctrlPortName,
        baudrate=9600,
        bytesize=8,
        timeout=1,
    )
    while True:
        match STATE:
            case -1:
                sentBytes = CTRLCHANNEL.write(START)
                print(sentBytes)
                STATE = 0
            case 0:
                print(DATACHANNEL.read(64))




if __name__ == "__main__":
    serial_read("/dev/tty.usbmodem1234561", "/dev/tty.usbmodem1234563")

                