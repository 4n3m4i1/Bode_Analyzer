import serial
import queue
PACKAGE_LENGTH = 16

def serial_read(queue, portName):
    with serial.Serial(
        port = portName,
        baudrate = 9600,
        bytesize = 8,
        timeout = 1,

    ) as streamInput:
        while True:
            if streamInput.in_waiting():
                infoPacket = streamInput.read(64)

                packetLength = infoPacket[0:PACKAGE_LENGTH - 1]