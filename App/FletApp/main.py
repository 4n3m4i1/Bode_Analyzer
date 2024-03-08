from generic_include import *
from assets.ref import *
import serial
import serial.tools.list_ports_osx as list_ports_osx
# import serial.tools.list_ports_windows as list_ports_windows
# import serial.tools.list_ports_linux as list_ports_linux
import atexit

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
if os == MACOS_STR: 
    for port in list_ports_osx.comports():
        port_selections.append(ft.dropdown.Option(str(port).split(".")[1].split("-")[0].strip()))


parametric_taps =[16,32,64,128,256]
taps_list =[]
size = len(parametric_taps)
for index in range(size):
    taps_list.append(ft.dropdown.Option(parametric_taps[index]))

frequency_ranges = ["High: 20Hz to 125kHz","Mid: 20Hz to 50kHz ","Low: 20Hz to 25kHz"]
range_list = []
size1 = len(frequency_ranges)
for index1 in range(size1):
    range_list.append(ft.dropdown.Option(frequency_ranges[index1]))



data_port = Port()
ctrl_port = Port()

init_graph = [0 for i in range(NUM_VALUES)]

FFT_real_queue = Queue()
FFT_converted_queue = Queue()

GraphEvent = Event()

def serial_read(dataPort: Port, ctrlPort: Port, data_Queue: Queue, page):
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
    def close_banner(e):
        page.banner.open = False
        page.update()
    
    x = 0
    while x < 1:
        is_conn = connected.wait()
        try:
            if dataPort.name == ctrlPort.name:
                raise serial.SerialException
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
            x = x + 1
            while True:
                is_set = GraphEvent.wait()

                # test_val = bytes([random.randint(0, 255) for _ in range(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)])
                match STATE:
                    case 1:
                        CTRLCHANNEL.write(SR_ACK)
                        HEADER = DATACHANNEL.read(HEADER_PACKET_LENGTH)
                        STATE = HH
                    case 2:
                        CTRLCHANNEL.write(SR_ACK)
                        # HHat = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                        # data_Queue.put(HHat, True)
                        STATE = FFR
                    case 3:
                        CTRLCHANNEL.write(SR_ACK)
                        FFR_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                        data_Queue.put(FFR_data, True)
                        # data_Queue.put(test_val, True)
                        STATE = FFI
                    case 4:
                        CTRLCHANNEL.write(SR_ACK)
                        # FFI_data = DATACHANNEL.read(DATA_PACKET_LENGTH * BYTES_PER_NUMBER)
                        # data_Queue.put(FFI_data, True)
                        STATE = IDLE
                    case 5:
                        CTRLCHANNEL.write(SR_ACK)
                        # IDLE_data = DATACHANNEL.read(CDC_PACKET_LENGTH * BYTES_PER_NUMBER)
                        STATE = H
        except serial.SerialException:
            page.banner = ft.Banner(
                bgcolor=ft.colors.AMBER_100,
                leading=ft.Icon(ft.icons.WARNING_AMBER_ROUNDED, color=ft.colors.AMBER, size=40),
                content=ft.Text(INVALID_PORTS_WARNING, color=ft.colors.BLACK),
                actions=[ft.TextButton(CLOSE_STR, on_click=close_banner),]
                )
            page.banner.open = True
            connected.clear()
            page.update()
        


def raw_data_to_float_converter(data_out_Queue: Queue, data_in_Queue: Queue):
    while True:
        is_set = GraphEvent.wait()
        try:
            if data_in_Queue.empty():
                raise Empty
            graph_data_raw = data_in_Queue.get(block=False)
            unpacked_graph_data = struct.iter_unpack('h', graph_data_raw)
            listed_graph_data = list(chain.from_iterable(unpacked_graph_data))
            converted_graph_data = Q15_to_float_array(listed_graph_data, 128)

            data_out_Queue.put(converted_graph_data, block=False)

        except Empty:
            continue

def update_graph(data_Queue: Queue, chart: MatplotlibChart, line, axis, fig):
    while True:
        is_set = GraphEvent.wait()
        try: 
            if data_Queue.empty():
                raise Empty
            data = data_Queue.get()
            line.set_ydata(data)
            
            plt.ylim(0, max(data))

            axis.draw_artist(line)
            fig.canvas.blit(fig.bbox)
            fig.canvas.flush_events()

            chart.update()

        except Empty:
            continue


def main(page: ft.Page):
    page.title = PAGE_TITLE

    def handle_start_button_clicked(e):
        if is_connected():
            connected.set()
            GraphEvent.set()
            page.update()
        else:
            handle_not_connected()

    def handle_stop_button_clicked(e):
        connected.clear()
        GraphEvent.clear()

    figure = plt.figure()
    ax = figure.add_subplot()
    line, = ax.plot(init_graph, animated=True)
    chart = MatplotlibChart(figure, expand=True)
    
    
    def is_connected():
        return True if data_port.name is not None and ctrl_port.name is not None else False
    
    def close_banner(e):
        page.banner.open = False
        page.update()
    def handle_not_connected():
        page.banner.open = True
        page.update()
    page.banner = ft.Banner(
            bgcolor=ft.colors.AMBER_100,
            leading=ft.Icon(ft.icons.WARNING_AMBER_ROUNDED, color=ft.colors.AMBER, size=40),
            content=ft.Text(NONE_PORTS_WARNING, color=ft.colors.BLACK),
            actions=[ft.TextButton(CLOSE_STR, on_click=close_banner),]
            )
    DataContainer = ft.Container(
        content=ft.Text('test'),
        image_src=BANDIT_LOGO_SRC,
        image_opacity=50
    )
    Controls = ft.Row(
        [ft.OutlinedButton(
        text=START_BUTTON_TEXT,
        width=150,
        on_click = handle_start_button_clicked,
    ), ft.OutlinedButton(
        text=STOP_BUTTON_TEXT,
        width=150,
        on_click = handle_stop_button_clicked
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
                data_port.set(f"{MACOS_PORT_PREFIX}{data_select.value}")
        page.update()
    def select_ctrl_port(e): #handle ctrl port selection
        match os:
            case "Darwin":
                ctrl_port.set(f"{MACOS_PORT_PREFIX}{ctrl_select.value}")
        page.update()

    data_select = ft.Dropdown(
            label=SELECT_DP_STR,
            options=port_selections,
            on_change=select_data_port,
        )
    ctrl_select = ft.Dropdown(
            label=SELECT_CP_STR,
            options=port_selections,
            on_change=select_ctrl_port
        )
    tap_select = ft.Dropdown(
            label=SELECT_TAP_STR,
            options=taps_list
        )
    freq_range_select = ft.Dropdown(
            label= SELECT_RANGE_STR,
            options=range_list
        )
    
    ConfigDisplay = ft.Column([
        ft.Text(CONFIG_INSTR),
        data_select,
        ctrl_select,
        tap_select,
        freq_range_select

        ]
    )

    SettingsSelection = ft.AlertDialog(
        modal=True,
        title=ft.Text(CONFIG_STR),
        content=ConfigDisplay,
        actions=[ft.TextButton(CLOSE_STR, on_click=close_modal)]
    )
    page.appbar = ft.AppBar(
        leading=ft.IconButton(ft.icons.BREAKFAST_DINING_OUTLINED),
        title=ft.Text(APPBAR_TITLE),
        bgcolor=ft.colors.SURFACE_VARIANT,
        actions=[
            ft.IconButton(ft.icons.SETTINGS, on_click=open_modal),
            ft.IconButton(icon=ft.icons.UNDO)
        ]
    )
    page.add(ft.Column([Controls,DataContainer]), chart)

    serial_reader = Thread(target=serial_read, args=(data_port, ctrl_port, FFT_real_queue, page))
    serial_reader.daemon = True
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue))
    data_converter_process.daemon = True
    update_graph_thread = Process(target=update_graph, args=(FFT_converted_queue, chart, line, ax, figure))
    update_graph_thread.daemon = True
    serial_reader.start()
    data_converter_process.start()
    update_graph_thread.start()

    
if __name__ == '__main__':
    ft.app(
        target=main,
        assets_dir='assets')
