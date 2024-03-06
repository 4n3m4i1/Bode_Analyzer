from generic_include import *
import serial
import serial.tools.list_ports_osx as list_ports_osx
import time

import random
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
# import numpy as np
from itertools import chain

# from scipy import fft
import struct 

import flet as ft
from flet.matplotlib_chart import MatplotlibChart

from multiprocess import Process, Queue, Event
from threading import Thread
from queue import Empty

# dataPort = "/dev/tty.usbmodem1234561"
# ctrlPort = "/dev/tty.usbmodem1234563"

dataPort = "/dev/tty.URT0"
ctrlPort = "/dev/tty.URT0"

NUM_VALUES = 128
port_selections = []
for port in list_ports_osx.comports():
    port_selections.append(ft.dropdown.Option(port))


parametric_taps =[16,32,64,128,256]
taps_list =[]
size = len(parametric_taps)
for index in range(size):
    taps_list.append(ft.dropdown.Option(parametric_taps[index]))



init_graph = [0 for i in range(NUM_VALUES)]

FFT_real_queue = Queue()
FFT_converted_queue = Queue()

GraphEvent = Event()

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
        bytesize = serial.EIGHTBITS,
        timeout = 1,
    )

    while True:
        is_set = GraphEvent.wait()

        # test_val = bytes([random.randint(0, 255) for _ in range(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)])
        match STATE:
            case 1:
                CTRLCHANNEL.write(b'a')
                # start = time.time()
                HEADER = DATACHANNEL.read(HEADER_PACKET_LENGTH)
                # end = time.time()
                # print(f"header: {end - start}")
                STATE = HH
            case 2:
                CTRLCHANNEL.write(b'a')
                # start = time.time()
                # HHat = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                # data_Queue.put(HHat, True)
                # end = time.time()
                # print(f"hhat: {end - start}")
                # print('hhat')
                STATE = FFR
            case 3:
                CTRLCHANNEL.write(b'a')
                # start = time.time()
                FFR_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                data_Queue.put(FFR_data, True)
                # end = time.time()
                # print(f"ffr: {end - start}")
                # data_Queue.put(test_val, True)
                # print(list(data_Queue.queue))
                # print('ffr')
                STATE = FFI
            case 4:
                CTRLCHANNEL.write(b'a')
                # start = time.time()
                # FFI_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                # data_Queue.put(FFI_data, True)
                # end = time.time()
                # print(f"ffi: {end - start}")
                STATE = IDLE
            case 5:
                CTRLCHANNEL.write(b'a')
                # start = time.time()
                # IDLE_data = DATACHANNEL.read(CDC_PACKET_LENGTH * BYTES_PER_NUMBER)
                # end = time.time()
                # print(f"idle: {end - start}")
                STATE = H

def raw_data_to_float_converter(data_out_Queue: Queue, data_in_Queue: Queue):
    while True:
        is_set = GraphEvent.wait()
        try:
            #print(f"data queue size: {data_in_Queue.qsize()}")
            graph_data_raw = data_in_Queue.get(block=False)
            unpacked_graph_data = struct.iter_unpack('h', graph_data_raw)
            listed_graph_data = list(chain.from_iterable(unpacked_graph_data))

            converted_graph_data = Q15_to_float_array(listed_graph_data, 128)

            # print(len(y_fft))
            # print(y_fft)
            data_out_Queue.put(converted_graph_data, block=False)
            # print(f"graph queue size: {data_out_Queue.qsize()}")
        except Empty:
            continue

def update_graph(data_Queue: Queue, chart: MatplotlibChart, line, axis, fig):
    while True:
        is_set = GraphEvent.wait()
        try: 

            data = data_Queue.get()
            line.set_ydata(data)
            # axis.autoscale_view()
            
            plt.ylim(0, max(data))

            axis.draw_artist(line)
            fig.canvas.blit(fig.bbox)
            fig.canvas.flush_events()

            # start = time.time()
            chart.update()
            # end = time.time()

            # print(end - start)
        except Empty:
            continue


def main(page: ft.Page):
    page.title = "BODE GUI TESTING"

    def handle_start_button_clicked(e):
        #running = True 
        GraphEvent.set()

    def handle_stop_button_clicked(e):
        GraphEvent.clear()

    figure = plt.figure()
    #plt.ylim(0, 10)
    ax = figure.add_subplot()
    line, = ax.plot(init_graph, animated=True)
    chart = MatplotlibChart(figure, expand=True)


    DataContainer = ft.Container(
        content=ft.Text('test'),
        image_src='/banditlogo.png',
        image_opacity=50
    )
    Controls = ft.Row([ft.OutlinedButton(
        text='Start',
        width=150,
        on_click = handle_start_button_clicked
    ), ft.OutlinedButton(
        text='Stop',
        width=150,
        on_click = handle_stop_button_clicked
        )])
    

    def open_modal(e):
        page.dialog = SettingsSelection
        SettingsSelection.open = True
        page.update()

    def close_modal(e):
        SettingsSelection.open = False
        page.update()

    ConfigDisplay = ft.Column([
        ft.Text('Select Port'),
        ft.Dropdown(
            options=port_selections
        ),

        ft.Text('Select number of Taps'),
        ft.Dropdown(
            options=taps_list
        ),

        ]
    )

    SettingsSelection = ft.AlertDialog(
        modal=True,
        title=ft.Text('Configurations'),
        content=ConfigDisplay,
        actions=[ft.TextButton('Cancel', on_click=close_modal), ft.TextButton('Submit')]
    )
    page.appbar = ft.AppBar(
        leading=ft.IconButton(ft.icons.BREAKFAST_DINING_OUTLINED),
        title=ft.Text('BANDIT'),
        bgcolor=ft.colors.SURFACE_VARIANT,
        actions=[
            ft.IconButton(ft.icons.SETTINGS, on_click=open_modal),
            ft.IconButton(icon=ft.icons.CLOSE)
        ]
    )
    page.add(ft.Column([Controls,DataContainer]), chart)

    serial_reader = Thread(target=serial_read, args=(dataPort, ctrlPort, FFT_real_queue))
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue))
    update_graph_thread = Process(target=update_graph, args=(FFT_converted_queue, chart, line, ax, figure))
    serial_reader.start()
    data_converter_process.start()
    update_graph_thread.start()


    
if __name__ == '__main__':
    ft.app(
        target=main,
        assets_dir='assets')
