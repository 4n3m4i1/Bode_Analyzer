import serial
CDC_PACKET_LENGTH = 64
DATA_PACKET_LENGTH = 128

def serial_read(dataPortName, ctrlPortName):
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

    CTRLCHANNEL.write(b'1')

    while True:
        idle = int(DATACHANNEL.read(CDC_PACKET_LENGTH).hex())

        if idle == 0:
            CTRLCHANNEL.write(b'1')

            header = DATACHANNEL.read(CDC_PACKET_LENGTH).hex(' ').split(' ')
            if 


if __name__ == "__main__":
    serial_read("/dev/tty.usbmodem1234561", "/dev/tty.usbmodem1234563")

                