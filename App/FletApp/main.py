from generic_include import *
from generic_include import BANDIT_SETTINGS_BYTES
from assets.ref import *
import serial
# from enum import IntEnum
# import serial.tools.list_ports_osx as list_ports_osx
# import serial.tools.list_ports_windows as list_ports_windows
# import serial.tools.list_ports_linux as list_ports_linux
import serial.tools.list_ports as list_ports
import platform
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import matplotlib.scale as scaler
import matplotlib.ticker as ticker
from itertools import chain
import struct 
import flet as ft
from flet.matplotlib_chart import MatplotlibChart
from flet import RouteChangeEvent, ViewPopEvent
from multiprocess import Process, Queue, Event
import multiprocessing
from multiprocessing import Queue, Lock
from threading import Thread
# from queue import Empty
import time
from bitstring import BitArray
import numpy as np
from scipy.interpolate import BSpline, make_interp_spline
os = platform.system()

# dataPort = None
# ctrlPort = None
start_event = Event()

settings_event = Event()

class Port():
    def __init__(self, portString = None):
        self.name = portString
    def set(self, port):
        self.name = port

NUM_VALUES = 128
port_selections = []
# if os == MACOS_STR: 
#     for port in list_ports_osx.comports():
#         port_selections.append(ft.dropdown.Option(str(port).split(".")[1]))
# elif os == LINUX_STR:
#     for port in list_ports_linux.comports():
#         port_selections.append(ft.dropdown.Option(str(port)))
# # elif os == WINDOWS_STR:
# #     for port in list_ports_windows.comports():
# #         port_selections.append(ft.dropdown.Option(str(port)))
# else: pass

for port in list_ports.comports():
    port_selections.append(ft.dropdown.Option(str(port)))
parametric_taps =[32,64,128,256, 512, 1024]
taps_list =[]
size = len(parametric_taps)
for index in range(size):
    taps_list.append(ft.dropdown.Option(key=parametric_taps[index]))

frequency_ranges = ["250K","125K","62.5K", "32.25K"]
range_list = []
size1 = len(frequency_ranges)
for index1 in range(size1):
    range_list.append(ft.dropdown.Option(key=index1, text=frequency_ranges[index1]))

data_port = Port()
ctrl_port = Port()

# init_graph = [0 for i in range(NUM_VALUES)]

GraphEvent = Event()
##########################################################################################SERIALREAD
def serial_read(dataPort: Port, ctrlPort: Port, data_Queue: Queue, settings_Queue: Queue, page, lock):
    BYTES_PER_NUMBER = 2
    CDC_PACKET_LENGTH = 64

    def close_banner(e):
        page.banner.open = False
        page.update()
    
    x = 0
    while x < 1:
        is_conn = start_event.wait()
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
            #send initial settings
            CTRLCHANNEL.write(b'~')
            print('start')
            CTRLCHANNEL.write(bytes(INIT_SETTINGS))
            # # # print(bytes(INIT_SETTINGS))
            # # #print(CTRLCHANNEL.read(64))
            setting = BitArray(hex=CTRLCHANNEL.read(SETTINGS_BF_LEN).hex())
            bandit_sett_bf = BitArray(hex=CTRLCHANNEL.read(4).hex())
            newfreqrange = BitArray(hex=CTRLCHANNEL.read(1).hex())
            newtaplen = BitArray(hex=CTRLCHANNEL.read(2).hex())
            new_err_limit = BitArray(hex=CTRLCHANNEL.read(2).hex())
            new_lms_offset = BitArray(hex=CTRLCHANNEL.read(2).hex())
            newlmsmax_attempts = BitArray(hex=CTRLCHANNEL.read(2).hex())
            newlearnin_rate = BitArray(hex=CTRLCHANNEL.read(2).hex())
            print(setting)
            print(bandit_sett_bf)
            print(newfreqrange)
            print(newtaplen)
            print(new_err_limit)
            print(new_lms_offset)
            print(newlmsmax_attempts)
            print(newlearnin_rate)

            # print(thing.bin)
            # CTRLCHANNEL.flush()
            while True:
                # print(DATACHANNEL.in_waiting)
                #if settings have been changed                
                if settings_event.is_set():
                    if settings_Queue.empty():
                        pass
                    else:
                        CTRLCHANNEL.write(b'~')
                        settings = settings_Queue.get(block=False)
                        print(settings)
                        CTRLCHANNEL.write(bytes(settings))
                        setting = BitArray(hex=CTRLCHANNEL.read(SETTINGS_BF_LEN).hex())
                        bandit_sett_bf = BitArray(hex=CTRLCHANNEL.read(4).hex())
                        newfreqrange = BitArray(hex=CTRLCHANNEL.read(1).hex())
                        newtaplen = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        new_err_limit = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        new_lms_offset = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        newlmsmax_attempts = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        newlearnin_rate = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        print(setting)
                        print(bandit_sett_bf)
                        print(newfreqrange)
                        print(newtaplen)
                        print(new_err_limit)
                        print(new_lms_offset)
                        print(newlmsmax_attempts)
                        print(newlearnin_rate)
                        settings_event.clear()
                else:
                    # print(DATACHANNEL.in_waiting)
                    if DATACHANNEL.in_waiting >= CDC_PACKET_LENGTH:
                        start = time.time()
                        header_data = DATACHANNEL.read(CDC_PACKET_LENGTH)
                        # print(header_data)
                        # print(len(header_data))
                        count = 0
                        CTRLCHANNEL.write(b'a')
                        num_samples = (header_data[3] * 256 + header_data[2]) * BYTES_PER_NUMBER
                        

                        # print(num_samples)
                        # print(DATACHANNEL.in_waiting)
                        # print(num_samples)
                        # print(header_data[2])
                        # print(header_data[3])
                        # print('while')
                        while DATACHANNEL.in_waiting < num_samples and count < 1000:
                            count = count + 1
                            # print(DATACHANNEL.in_waiting)
                        
                        f_data = DATACHANNEL.read(num_samples)
                        end = time.time()
                        # print(f_data)
                        print(f'{end - start} second transmition')
                        lock.acquire()
                        data_Queue.put(f_data, block=False)
                        lock.release()
        except serial.SerialException:
            page.banner = ft.Banner(
                bgcolor=ft.colors.AMBER_100,
                leading=ft.Icon(ft.icons.WARNING_AMBER_ROUNDED, color=ft.colors.AMBER, size=40),
                content=ft.Text(INVALID_PORTS_WARNING, color=ft.colors.BLACK),
                actions=[ft.TextButton(CLOSE_STR, on_click=close_banner),]
                )
            page.banner.open = True
            start_event.clear()
            page.update()
        except OSError:
            x = x - 1
            start_event.clear()
            # page.update()

##########################################################################################DATACONVERTER
def raw_data_to_float_converter(data_out_Queue: Queue, data_in_Queue: Queue, lock):
    # is_set = GraphEvent.wait()
    while True:
        is_set = GraphEvent.wait()
        # lock.acquire()
        try:
            graph_data_raw = data_in_Queue.get(block=False)
            # print(len(graph_data_raw))
            # lock.release()
            # print(graph_data_raw)
            # print(len(graph_data_raw))
            unpacked_graph_data = struct.iter_unpack('h', graph_data_raw)
            listed_graph_data = list(chain.from_iterable(unpacked_graph_data))
            # print(len(listed_graph_data))
            converted_graph_data = Q15_to_float_array(listed_graph_data, len(listed_graph_data))
            # print(converted_graph_data)
            data_out_Queue.put(converted_graph_data, block=False)
        except Exception as e:
            # lock.release()
            pass
##########################################################################################UPDATEGRAPH
def update_graph(data_Queue: Queue, chart: MatplotlibChart, line: matplotlib.lines.Line2D, frange_queue: Queue, axis, fig):
    # is_set = GraphEvent.wait()
    frange = 0
    print('start graph')
    while True:
        # print(' ')
        # print(frange)
        is_set = GraphEvent.wait()
        try:
            data = data_Queue.get()
            # print(data)
            # print(data)
            # print(len(data))
            line.set_data(np.arange(len(data)), data)
            if frange_queue.empty():
                if frange == 0:
                    frange = 250e3
                else:
                    pass
            else:

                frange = frange_queue.get()

            if data:
                if max(data) != 0:
                    print("Valid Data Print!!")
                    #print(line.get_data())
                    plt.xlim(1e-15, len(data))
                    ticks_x = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format((x / len(data)) * frange))
                    plt.ylim(min(data), max(data) + 1e-13)
                    # plt.xlim(min(float), le)
                    # axis.set_xbound
                    axis.xaxis.set_major_formatter(ticks_x)
                    # axis.xaxis.
                    axis.draw_artist(line)
                    print('oop')
                    fig.canvas.blit(fig.bbox)  # I added this
                    fig.canvas.flush_events()
            # print('step')
                    print('here')
                    chart.update()
                    print('huh')
            # print('step2')
                else:
                    print("Data All Zero!!")
                
            else:
                pass
        except Exception as e:

            print(e)
            pass
            
##########################################################################################MAIN
def main(page: ft.Page):
    page.title = PAGE_TITLE
    page.route = "/"
    page.bgcolor = '#e3e3e3'
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER ## test
    page.theme = ft.Theme(
    color_scheme=ft.ColorScheme(
        primary=ft.colors.BLACK,   
    )
)
    temp_settings = INIT_SETTINGS

    pool = multiprocessing.Pool(processes=1)
    m = multiprocessing.Manager()
    FFT_real_queue = m.Queue()
    FFT_converted_queue = m.Queue()
    Settings_Queue = m.Queue()
    FRANGE_queue = m.Queue()
    lock = Lock()

    def route_change(e: RouteChangeEvent) -> None:
        if page.route == "/about":
            page.views.append(
                ft.View(
                    route = "/about",
                    controls = [
                        ft.AppBar(title=ft.Text(ABOUT_TITLE), bgcolor=ft.colors.SURFACE_VARIANT), 
                        logo, about_header, mission_statement, meet_team
                    ],   
                )
            )
        page.update()


    def view_pop(view):
        page.views.pop()
        top_view = page.views[-1]
        page.go(top_view.route)

    page.on_route_change = route_change
    page.on_view_pop = view_pop
    page.go(page.route)

    def handle_start_button_clicked(e):
        if is_connected():
            start_event.set()
            GraphEvent.set()
            page.update()
            temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTORUN.value] = 1
            temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTOSEND.value] = 1
            Settings_Queue.put(temp_settings)
            settings_event.set()
        else:
            handle_not_connected()

    def handle_stop_button_clicked(e):
        start_event.clear()
        GraphEvent.clear()
        stop_settings = temp_settings
        stop_settings[BANDIT_SETTINGS_BYTES.USBBSRX_SINGLE_SHOT.value] = 0
        stop_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTORUN.value] = 0
        stop_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTOSEND.value] = 0

        Settings_Queue.put(stop_settings)

        settings_event.set()

    
        
    ## Graph configurations
    #figure = plt.figure()
    figure = plt.figure(figsize=(15,7))
    ax = figure.add_subplot(111)
    line, = ax.plot(init_graph, animated=True,)
    ax.grid()

    line.set_color('#F55BB0')
    chart = MatplotlibChart(figure, expand=True,)
    #ax.set_facecolor('#e3e3e3')
    figure.set_facecolor('#e3e3e3')
    ax.set_xscale(PLOT_XUNIT)
    ax.set_yscale(PLOT_YUNIT)
    ax.set_title(PLOT_TITLE, fontsize = 18)
    ax.set_xlabel(PLOT_XLABEL, fontsize = 18) 
    ax.set_ylabel(PLOT_YLABEL, fontsize = 18)
    
    
    
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
    
    ## Content for the about us page
    logo = ft.Container(
        ft.Image(
            src= BANDIT_LOGO_SRC,
            width=150,
            height=150,
            fit=ft.ImageFit.CONTAIN,
        ),

        alignment = ft.alignment.center
    )
    
    about_header = ft.Container(
        ft.Text("The Bode Bandits", theme_style=ft.TextThemeStyle.DISPLAY_LARGE),
        )
    
    mission_statement = ft.Container(
        ft.Text("Bode Analysis N’ Display of Instrument Testing") #TBD
        )
    
    meet_team =  ft.Container(
        ft.Text("Meet the Team", theme_style=ft.TextThemeStyle.DISPLAY_MEDIUM),
        )
    
    start_container = ft.Container (

        content= ft.OutlinedButton(
        text=START_BUTTON_TEXT,
        width=150,
        on_click = handle_start_button_clicked,
    ),
        bgcolor='#ecd4df',
        border_radius=20,
    )

    stop_container = ft.Container (
        content= ft.OutlinedButton(
        text=STOP_BUTTON_TEXT,
        width=150,
        on_click = handle_stop_button_clicked,
    ),
        #bgcolor='#e895c0',
        bgcolor='#c27ba0',
        border_radius=20,
    )

    # change bit at index 3
    def toggle_wgn(e):
        if wgn_switch.label == WGN_LABEL_ON:
            temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_WGN_ALWAYS_ON.value] = 0
            #print("off")
        else :
            #print("on")
            temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_WGN_ALWAYS_ON.value] = 1

        wgn_switch.label = (
            WGN_LABEL_OFF if temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_WGN_ALWAYS_ON.value] == 0 else WGN_LABEL_ON
        )
        print(wgn_switch.label)
        print(temp_settings)

        page.update()
    def toggle_single_shot(e):
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_SINGLE_SHOT.value] = int(single_shot_switch.value)
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTORUN.value] = int(not temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_SINGLE_SHOT.value])

    def toggle_auto_run(e):
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTORUN.value] = int(auto_run_switch.value)
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_SINGLE_SHOT.value] = int(not temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTORUN.value])

    def toggle_raw_request(e):
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_RAW_RQ.value] = int(raw_requect_switch.value)
    

    def toggle_time_d(e):
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_TIME_DOMAIN_DATA.value] = int(time_domain_switch.value)



    wgn_switch = ft.Switch(label= WGN_LABEL_OFF, on_change=toggle_wgn)

    single_shot_switch = ft.Switch(label = SINGLE_SHOT_LABEL, on_change=toggle_single_shot)

    auto_run_switch = ft.Switch(label = AUTO_RUN_LABEL, on_change=toggle_auto_run)

    raw_requect_switch = ft.Switch(label = RAW_REQUEST_LABEL, on_change=toggle_raw_request)

    time_domain_switch = ft.Switch(label = TIME_DOMAIN_REQUEST_LABEL, on_change=toggle_time_d)

    
#######################################################
    config_table = ft.DataTable(
        border=ft.border.all(1, "black"),
        bgcolor = 'white',

            columns=[
                ft.DataColumn(ft.Text("Control Port")),
                ft.DataColumn(ft.Text("Data Port")),
                ft.DataColumn(ft.Text("Taps")),
                ft.DataColumn(ft.Text("Frequency Range")),
            ],
            rows=[
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("Not Selected")),
                        ft.DataCell(ft.Text("Not Selected")),
                        ft.DataCell(ft.Text("Not Selected")),
                        ft.DataCell(ft.Text("Not Selected")),
                    ],
            ),
            ],
        )
    

    Controls = ft.Row(
        [start_container
        ,stop_container,
        ] 
        )
    
    # switches = ft.Row(
    #     [ wgn_switch
    #     ] 
    #     )
    
    
    
    def open_modal(e): # handle settings modal opening
        page.dialog = SettingsSelection
        SettingsSelection.open = True
        page.update()

    def close_modal(e): #handle settings modal closing
        SettingsSelection.open = False  
        if max(temp_settings) != 0 and start_event.is_set(): 
            Settings_Queue.put(temp_settings)
            settings_event.set()
        updated_table = ft.DataTable(
            border=ft.border.all(1, "black"),
            bgcolor = 'white',

                columns=[
                    ft.DataColumn(ft.Text("Control Port")),
                    ft.DataColumn(ft.Text("Data Port")),
                    ft.DataColumn(ft.Text("Taps")),
                    ft.DataColumn(ft.Text("Frequency Range")),
                ],
                rows=[
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(data_select.value)),
                            ft.DataCell(ft.Text(ctrl_select.value)),
                            ft.DataCell(ft.Text(tap_select.value)) if tap_select.value else ft.DataCell(ft.Text(parametric_taps[0])),
                            ft.DataCell(ft.Text(frequency_ranges[int(freq_range_select.value)]))
                                if freq_range_select.value
                                else ft.DataCell(ft.Text(frequency_ranges[0]))
                        ],
                        
                ),
                ],
            )

        page.controls.clear()
        page.add(ft.Column([Controls]),updated_table,chart)

        page.update()

    def select_data_port(e): #handle data port selection
        if os == MACOS_STR:
            portName = data_select.value.split("-")[0].strip()
            data_port.set(portName)
        elif os == LINUX_STR:
            portName = data_select.value.split("-")[0].strip()
            data_port.set(portName)
        page.update()
    def select_ctrl_port(e): #handle ctrl port selection
        if os == MACOS_STR:
            portName = ctrl_select.value.split("-")[0].strip()
            ctrl_port.set(portName)
        elif os == LINUX_STR:
            portName = ctrl_select.value.split("-")[0].strip()
            ctrl_port.set(portName)
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
    def change_LR(e):
        bytes_tmp = int(learning_rate_slider.value).to_bytes(2, 'little')
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_LEARNING_RATE_LSB.value] = bytes_tmp[0]
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_LEARNING_RATE_MSB.value] = bytes_tmp[1]
        print(temp_settings)

    def change_OFS(e):
        bytes_tmp = int(offset_delay_slider.value)

        taplen = temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_TAPLEN_LSB.value]
        taplen = taplen + temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_TAPLEN_MSB.value] * 256

        dividy = MAX_TAPS / taplen;

        bytes_tmp = bytes_tmp / dividy;

        bytes_tmp = int(bytes_tmp).to_bytes(2, 'little', signed = True)

        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_OFFSET_LSB.value] = bytes_tmp[0]
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_OFFSET_MSB.value] = bytes_tmp[1]
        print(temp_settings)

    def select_tap_length(e):
        bytes_tmp = int(tap_select.value).to_bytes(2, 'little')
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_TAPLEN_LSB.value] = bytes_tmp[0]
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_TAPLEN_MSB.value] = bytes_tmp[1]
        
        change_OFS(e)

    def select_frange(e):
        tmp = int(freq_range_select.value)
        temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_F_FRANGE.value] = tmp
        FRANGE_queue.put(FRANGE_VAL[tmp])
        print(temp_settings)

    tap_select = ft.Dropdown(
            label=SELECT_TAP_STR,
            options=taps_list,
            on_change=select_tap_length
        )
    
    freq_range_select = ft.Dropdown(
            label= SELECT_RANGE_STR,
            options=range_list,
            on_change=select_frange

        )
    
    learning_rate_slider = ft.Slider(
        label = "{value}",
        min=0x01, 
        max=0x7FFF, 
        divisions=32767, 
        on_change=change_LR
        )
    
    offset_delay_slider = ft.Slider(
        label = "{value}",
        min = -512, 
        max = 512, 
        divisions = 1024, 
        on_change=change_OFS
        )
    
    ConfigDisplay = ft.Column(controls=[
        ft.Text(CONFIG_INSTR),
        data_select,
        ctrl_select,
        tap_select,
        freq_range_select,
        wgn_switch,
        single_shot_switch,
        auto_run_switch,
        raw_requect_switch,
        time_domain_switch,
        learning_rate_slider,
        offset_delay_slider
        ],
        scroll=ft.ScrollMode.ALWAYS

    )

    SettingsSelection = ft.AlertDialog(
        modal=True,
        title=ft.Text(CONFIG_STR),
        content=ConfigDisplay,
        actions=[ft.TextButton(CLOSE_STR, on_click=close_modal)],
    )
    page.appbar = ft.AppBar(
        leading=ft.IconButton(ft.icons.BREAKFAST_DINING_OUTLINED),

        title=ft.Text(APPBAR_TITLE),
        #bgcolor=ft.colors.SURFACE_VARIANT,
        #bgcolor= '#f08dbf',
        bgcolor= '#d5a6bd',
        actions=[
            ft.TextButton(ABOUT_US_TEXT, on_click = lambda _: page.go("/about")), 
            ft.IconButton(ft.icons.SETTINGS, on_click=open_modal),
            ft.IconButton(icon=ft.icons.UNDO)
        ]
    )

    #page.add(ft.Column([Controls]) ,chart ) 
    page.add(ft.Column([Controls]),config_table,chart)

    serial_reader = Thread(target=serial_read, args=(data_port, ctrl_port, FFT_real_queue, Settings_Queue, ft.page, lock))
    serial_reader.daemon = True
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue, lock))
    data_converter_process.daemon = True
    print('try')
    update_graph_thread = Thread(target=update_graph, args=(FFT_converted_queue, chart, line, FRANGE_queue, ax, figure))
    # update_graph_thread.nice
    update_graph_thread.daemon = True
    update_graph_thread.start()
    serial_reader.start()
    data_converter_process.start()
    
if __name__ == '__main__':
    ft.app(
        target=main,
        assets_dir='assets')