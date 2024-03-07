from generic_include import *
import serial
import serial.tools.list_ports_osx as list_ports_osx
# import serial.tools.list_ports_windows as list_ports_windows
# import serial.tools.list_ports_linux as list_ports_linux

import time
import platform

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

os = platform.system()

# dataPort = None
# ctrlPort = None
MacOS = "Darwin"
connected = Event()
class Port():
    def __init__(self, portString = None):
        self.name = portString
    def set(self, port):
        self.name = port


# dataPort = "/dev/tty.usbmodem1234561"
# ctrlPort = "/dev/tty.usbmodem1234563"

# dataPort = "/dev/tty.URT0"
# ctrlPort = "/dev/tty.URT0"

NUM_VALUES = 128
port_selections = []
if os == MacOS: 
    for port in list_ports_osx.comports():
        port_selections.append(ft.dropdown.Option(str(port).split(".")[1].split("-")[0].strip()))


parametric_taps =[16,32,64,128,256]
taps_list =[]
size = len(parametric_taps)
for index in range(size):
    taps_list.append(ft.dropdown.Option(parametric_taps[index]))


data_port = Port()
ctrl_port = Port()

init_graph = [0 for i in range(NUM_VALUES)]

FFT_real_queue = Queue()
FFT_converted_queue = Queue()

GraphEvent = Event()

def serial_read(dataPort: Port, ctrlPort: Port, data_Queue: Queue):
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

    is_conn = connected.wait()

    DATACHANNEL = serial.Serial(
        port = dataPort.name,
        baudrate = 9600,
        bytesize = serial.EIGHTBITS,
        timeout = 1,

    )
    CTRLCHANNEL = serial.Serial(
        port= ctrlPort.name,
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
        connected.set()
        GraphEvent.set()
        page.update()

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
    Controls = ft.Row(
        [ft.OutlinedButton(
        text='Start',
        width=150,
        on_click = handle_start_button_clicked
    ), ft.OutlinedButton(
        text='Stop',
        width=150,
        on_click = handle_stop_button_clicked
        ),
        ft.Icon(
        name = ft.icons.SIGNAL_WIFI_4_BAR if connected.is_set() else ft.icons.SIGNAL_WIFI_OFF, 
        color = ft.colors.GREEN_ACCENT if connected.is_set() else ft.colors.RED_ACCENT
        ),
        ])
    

    def open_modal(e): # handle settings modal opening
        page.dialog = SettingsSelection
        SettingsSelection.open = True
        page.update()

    def close_modal(e): #handle settings modal closing
        SettingsSelection.open = False
        page.update()

    def select_data_port(e): #handle data port selection
        match os:
            case "Darwin":
                data_port.set(f"/dev/tty.{data_select.value}")

    def select_ctrl_port(e): #handle ctrl port selection
        ctrl_port.set(f"/dev/tty.{ctrl_select.value}")

    data_select = ft.Dropdown(
            label="Select Data Port",
            options=port_selections,
            on_change=select_data_port,
        )
    ctrl_select = ft.Dropdown(
            label="Select Control Port",
            options=port_selections,
            on_change=select_ctrl_port
        )
    tap_select = ft.Dropdown(
            label="Select Number of Taps",
            options=taps_list
        )
    ConfigDisplay = ft.Column([
        data_select,
        ctrl_select,
        tap_select,

        ]
    )

    SettingsSelection = ft.AlertDialog(
        modal=True,
        title=ft.Text('Configurations'),
        content=ConfigDisplay,
        actions=[ft.TextButton('Close', on_click=close_modal)]
    )
    page.appbar = ft.AppBar(
        leading=ft.IconButton(ft.icons.BREAKFAST_DINING_OUTLINED),
        title=ft.Text('BANDIT'),
        bgcolor=ft.colors.SURFACE_VARIANT,
        actions=[
            ft.IconButton(ft.icons.SETTINGS, on_click=open_modal),
            ft.IconButton(icon=ft.icons.UNDO)
        ]
    )
    page.add(ft.Column([Controls,DataContainer]), chart)

    serial_reader = Thread(target=serial_read, args=(data_port, ctrl_port, FFT_real_queue))
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue))
    update_graph_thread = Process(target=update_graph, args=(FFT_converted_queue, chart, line, ax, figure))
    serial_reader.start()
    data_converter_process.start()
    update_graph_thread.start()


    
if __name__ == '__main__':
    ft.app(
        target=main,
        assets_dir='assets')
