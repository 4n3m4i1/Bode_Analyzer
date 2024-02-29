from generic_include import *
import serial

import asyncio
import random
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import numpy as np
from itertools import chain

from scipy import fft
import struct 

import flet as ft
from flet.matplotlib_chart import MatplotlibChart

from multiprocessing import Queue
from threading import Thread
from queue import Empty
dataPort = "/dev/tty.usbmodem1234561"
ctrlPort = "/dev/tty.usbmodem1234563"

NUM_VALUES = 128

init_graph = [0 for i in range(NUM_VALUES)]

FFT_real_queue = Queue()
FFT_converted_queue = Queue()

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
        test_val = bytes([random.randint(0, 255) for _ in range(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)])
        match STATE:
            case 1:
                CTRLCHANNEL.write(b'a')
                HEADER = DATACHANNEL.read(HEADER_PACKET_LENGTH)
                STATE = HH
            case 2:
                CTRLCHANNEL.write(b'a')
                HHat = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                # data_Queue.put(HHat, True)
                # print('hhat')
                STATE = FFR
            case 3:
                CTRLCHANNEL.write(b'a')
                FFR_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                # data_Queue.put(FFR_data, True)
                data_Queue.put(test_val, True)
                # print(list(data_Queue.queue))
                # print('ffr')
                STATE = FFI
            case 4:
                CTRLCHANNEL.write(b'a')
                FFI_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                # data_Queue.put(FFI_data, True)
                
                STATE = IDLE
            case 5:
                CTRLCHANNEL.write(b'a')
                IDLE_data = DATACHANNEL.read(CDC_PACKET_LENGTH * BYTES_PER_NUMBER)
                STATE = H


def raw_data_to_float_converter(data_out_Queue: Queue, data_in_Queue: Queue):
    while True:
        try:
            graph_data_raw = data_in_Queue.get(block=False)
            unpacked_graph_data = struct.iter_unpack('h', graph_data_raw)
            listed_graph_data = list(chain.from_iterable(unpacked_graph_data))

            converted_graph_data = Q15_to_float_array(listed_graph_data, 128)

            # print(len(y_fft))
            # print(y_fft)
            data_out_Queue.put(converted_graph_data, block=False)

        except Empty:
            continue

def update_graph(data_Queue: Queue, chart: MatplotlibChart, line, axis):
    while True:
        try: 
            data = data_Queue.get()
            line.set_ydata(data)
            axis.autoscale_view()
            plt.ylim(0, max(data))
            plt.draw()


            chart.update()
        except Empty:
            continue


        
def main(page: ft.Page):
    page.title = "BODE GUI TESTING"


    figure = plt.figure()
    # plt.ylim(0, 10)
    ax = figure.add_subplot()
    line, = ax.plot(init_graph)
    chart = MatplotlibChart(figure, isolated=True, expand=True)
    serial_reader = Thread(target=serial_read, args=(dataPort, ctrlPort, FFT_real_queue))
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue))
    update_graph_thread = Thread(target=update_graph, args=(FFT_converted_queue, chart, line, ax))
    def handle_start_button_clicked(e):
        # running = True
        serial_reader.start()
        data_converter_process.start()
        update_graph_thread.start()

        
    page.add(ft.OutlinedButton(
        width=150,
        on_click=handle_start_button_clicked
    ), 
    chart)



    

ft.app(target=main)
