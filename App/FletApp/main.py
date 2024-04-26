import webbrowser
from generic_include import *
from generic_include import BANDIT_SETTINGS_BYTES
from assets.ref import *
import serial
import serial.tools.list_ports
# import serial.tools.list_ports_osx as list_ports_osx
# import serial.tools.list_ports_windows as list_ports_windows
# import serial.tools.list_ports_linux as list_ports_linux
# import serial.tools.list_ports as list_ports
import platform
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
from itertools import chain
import struct 
import flet as ft
from flet.matplotlib_chart import MatplotlibChart
from flet import RouteChangeEvent, ViewPopEvent
from multiprocess import Process, Event
from multiprocessing import Queue
import multiprocessing
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
GraphEvent = Event()
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

for port in serial.tools.list_ports.comports():
    print(port)
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

##########################################################################################SERIALREAD
def serial_read(dataPort: Port, ctrlPort: Port, data_Queue: Queue, settings_Queue: Queue, page):
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
            setting = BitArray(hex=CTRLCHANNEL.read(10).hex())
            rest = BitArray(hex=CTRLCHANNEL.read(4).hex())
            rest2 = BitArray(hex=CTRLCHANNEL.read(1).hex())
            rest3 = BitArray(hex=CTRLCHANNEL.read(2).hex())
            rest4 = BitArray(hex=CTRLCHANNEL.read(2).hex())
            print(setting)
            print(rest)
            print(rest2)
            print(rest3)
            print(rest4)
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
                        # print(settings)
                        CTRLCHANNEL.write(bytes(settings))
                        setting = BitArray(hex=CTRLCHANNEL.read(10).hex())
                        rest = BitArray(hex=CTRLCHANNEL.read(4).hex())
                        rest2 = BitArray(hex=CTRLCHANNEL.read(1).hex())
                        rest3 = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        rest4 = BitArray(hex=CTRLCHANNEL.read(2).hex())
                        print(setting)
                        print(rest)
                        print(rest2)
                        print(rest3)
                        print(rest4)
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
                        data_Queue.put(f_data)
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
##########################################################################################DATACONVERTER
def raw_data_to_float_converter(data_out_Queue: Queue, data_in_Queue: Queue):
    # is_set = GraphEvent.wait()
    while True:
        is_set = GraphEvent.wait()
        if data_in_Queue.empty():
            pass
            
        else:
            graph_data_raw = data_in_Queue.get(block=False)
            # print(len(graph_data_raw))
            unpacked_graph_data = struct.iter_unpack('h', graph_data_raw)
            listed_graph_data = list(chain.from_iterable(unpacked_graph_data))
            # print(len(listed_graph_data))
            converted_graph_data = Q15_to_float_array(listed_graph_data, len(listed_graph_data))
            # print(converted_graph_data)
            data_out_Queue.put(converted_graph_data, block=False)
##########################################################################################UPDATEGRAPH
def update_graph(data_Queue: Queue, chart: MatplotlibChart, line: matplotlib.lines.Line2D, frange_queue: Queue, axis, fig):
    # is_set = GraphEvent.wait()
    frange = 0
    while True:
        # print(frange)
        is_set = GraphEvent.wait()
        if data_Queue.empty():
            pass
        else:
            data = data_Queue.get()
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
                    # print(line.get_data())
                    plt.xlim(1e-15, frange)
                    plt.ylim(1e-15, max(data) + 0.01)
                    # plt.xlim(min(float), le)
                    # axis.set_xbound
                    axis.draw_artist(line)
                    fig.canvas.blit(fig.bbox)
                    fig.canvas.flush_events()
            # print('step')
                    chart.update()
            # print('step2')
                else:
                    print("Data All Zero!!")
                
            else:
                pass
            
##########################################################################################MAIN
def main(page: ft.Page):
    page.title = PAGE_TITLE
    page.route = "/"
    page.bgcolor = PAGE_BG_COLOR
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER ## test

    page.window_width = 1000     
    #page.window_height = 1000  

    page.theme = ft.Theme(
    color_scheme=ft.ColorScheme(
        primary=ft.colors.BLACK,   
    )
)
    
            ## Graph configurations
    #figure = plt.figure()
    figure = plt.figure(figsize=(15,7))
    #figure = plt.figure(figsize=(30,7))
    ax = figure.add_subplot()
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

    pool = multiprocessing.Pool(processes=1)
    m = multiprocessing.Manager()
    FFT_real_queue = m.Queue()
    FFT_converted_queue = m.Queue()
    Settings_Queue = m.Queue()

    FRANGE_queue = m.Queue()

    serial_reader = Thread(target=serial_read, args=(data_port, ctrl_port, FFT_real_queue, Settings_Queue, ft.page))
    serial_reader.daemon = True
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue))
    data_converter_process.daemon = True
    update_graph_thread = pool.apply_async(update_graph, (FFT_converted_queue, chart, line, FRANGE_queue, ax, figure))
    update_graph_thread.daemon = True
    serial_reader.start()
    data_converter_process.start()
    # update_graph_thread.start()
    temp_settings = INIT_SETTINGS

    def open_youtube(e):
        youtube_url = "https://youtu.be/heG2l5tyS7A?si=Tvam4DrzBTFw0_lb" 
        webbrowser.open(youtube_url)
                        

                        
    def route_change(e: RouteChangeEvent) -> None:
        if page.route == "/about":
            page.vertical_alignment = ft.MainAxisAlignment.CENTER
            page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
            page.theme = ft.Theme(
                color_scheme_seed=LIGHT_PINK,
            )       

            youtube_button = ft.ElevatedButton(
                text="Watch Here",
                on_click=open_youtube,
                icon=ft.icons.TV
            )   
            
            controls_about = [
                ft.AppBar(title=ft.Text(ABOUT_TITLE), bgcolor=BANNER_COLOR, actions=[youtube_button]),
                logo,overview_title, overview_statement, specs_title, specs_description, technologies_title, technologies_description,
                meet_team,  meet_ari #team_pictures, meet_ari,
            ] 
            alignment=ft.MainAxisAlignment.CENTER,
            
            page.views.append(
                ft.View(
                    route="/about",
                    controls = controls_about 
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
        else:
            handle_not_connected()

    def handle_stop_button_clicked(e):
        start_event.clear()
        GraphEvent.clear()

    
        

    
    
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
        content = ft.Image(
            src= BANDIT_LOGO_SRC,
            width=150,
            height=150,
            fit=ft.ImageFit.CONTAIN,
        ),
        alignment = ft.alignment.center,
    )
        
    overview_title = ft.Container(
        content = ft.Text("Project Overview", theme_style=ft.TextThemeStyle.TITLE_LARGE),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )

    overview_statement = ft.Container(
        ft.Text("Bode Analysis N’ Display of Instrument Testing (BANDIT) is a small, portable, and inexpensive Frequency Response Analyzer aimed at providing rapid assessments of a system’s transfer function. User selectable resolutions and analysis ranges, as well as output visualization, are provided by a comprehensive Graphical User Interface. BANDIT is capable of in depth self-calibration routines correcting for board to board component tolerances and voltage supply fluctuations, ensuring performance across devices without expensive component binning.", theme_style=ft.TextThemeStyle.BODY_MEDIUM),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
    specs_title =  ft.Container(
        ft.Text("Key Specifications", theme_style=ft.TextThemeStyle.TITLE_LARGE),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
    specs_description = ft.Container(
        ft.DataTable(
            width=2000,
            bgcolor = LIGHT_PINK,
            border = ft.border.all(2,DARK_PINK),
            #border_radius = 10,
            vertical_lines=ft.border.BorderSide(width=2, color=DARK_PINK),
            horizontal_lines=ft.border.BorderSide(width=2, color=DARK_PINK),
            sort_column_index=0,
            sort_ascending=True,
            heading_row_color=ft.colors.BLACK12,
            heading_row_height=0,
            data_row_max_height=25,
            data_row_color= LIGHT_PINK,
            column_spacing= 200,
            columns = [
                ft.DataColumn(ft.Text("")),
                ft.DataColumn(ft.Text(""),numeric=False),

            ],
            rows=[
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("USB Powered")),
                         ft.DataCell(ft.Text("Operates across entire USB 2.0 voltage range")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("Input")),
                        ft.DataCell(ft.Text("Direct from output of GWN fed SUT (4Vpk-pk Signal max)")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("Output:")),
                         ft.DataCell(ft.Text("Gaussian White Noise (GWN), input for System Under Test")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("Digital Signal Processing")),
                         ft.DataCell(ft.Text("Dynamic Downsampling, Least Mean Squared (LMS) Adaptive Filter, Cooley-Tukey FFT, Fully Q15 fixed point routines. Optimized for dual-core ARM M0+")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("Greater than 1 HZ update Rate")),
                         ft.DataCell(ft.Text("")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("GUI")),
                         ft.DataCell(ft.Text("Multiprocessing cross-platform Python script")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("High speed USB Communications Device Class (CDC) interface")),
                         ft.DataCell(ft.Text("")),
                    ],
                ),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text("Analysis Ranges")),
                         ft.DataCell(ft.Text("20Hz - 31kHz, 20Hz - 62kHz, 20Hz - 125kHz")),                
                    ],
                ),
            ],
        ),
        alignment=ft.alignment.center
    )
    
    technologies_title =  ft.Container(
        ft.Text("Key Technologies", theme_style=ft.TextThemeStyle.TITLE_LARGE),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
    technologies_description =  ft.Container(
        ft.Text("Developed: Multiprocessing GUI, Multicore Embedded Firmware, DSP Processing Stack, Device Enclosure\n                                                            Procured: PCB Assembly", theme_style=ft.TextThemeStyle.BODY_MEDIUM),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    

    meet_team =  ft.Container(
        ft.Text("Meet the Bode Bandits", theme_style=ft.TextThemeStyle.TITLE_LARGE),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
    team_pictures = ft.Container(
       ft.Image(
            src= ARI_IMAGE,
            width=200,
            height=200
        ),
            alignment = ft.alignment.center

   
    )

    meet_ari = ft.Container(
        ft.ElevatedButton("Arianna-Sarahi Bergado", on_click=lambda _: page.go("/ari")),
        ft.ElevatedButton("Exit", on_click=lambda _:page.go("/about"))
    )
        
    
    start_container = ft.Container (

        content= ft.OutlinedButton(
        text=START_BUTTON_TEXT,
        width=150,
        on_click = handle_start_button_clicked,
    ),
        bgcolor= START_BUTTON_COLOR,
        border_radius=20,
    )

    stop_container = ft.Container (
        content= ft.OutlinedButton(
        text=STOP_BUTTON_TEXT,
        width=150,
        on_click = handle_stop_button_clicked,
    ),
        #bgcolor='#e895c0',
        bgcolor= STOP_BUTTON_COLOR,
        border_radius=20,
    )

    # change bit at index 3
    def toggle_wgn(e):
        # if wgn_switch.label == WGN_LABEL_ON:
        #     temp_settings[3] = 0
        #     #print("off")
        # else :
        #     #print("on")
        #     temp_settings[3] = 1

        # wgn_switch.label = (
        #     WGN_LABEL_OFF if temp_settings[3] == 0 else WGN_LABEL_ON
        # )

        temp_settings[3] = 1 if wgn_switch.value == 1 else 0
        print(wgn_switch.label)
        print(temp_settings)

        page.update()
    def toggle_single_shot(e):
        temp_settings[12] = int(single_shot_switch.value)
        temp_settings[1] = int(not temp_settings[12])

    def toggle_auto_run(e):
        temp_settings[1] = int(auto_run_switch.value)
        temp_settings[12] = int(not temp_settings[1])

    def toggle_raw_request(e):
        temp_settings[10] = int(raw_requect_switch.value)
    

    def toggle_time_d(e):
        temp_settings[11] = int(time_domain_switch.value)


    wgn_switch = ft.Switch(label= WGN_LABEL, on_change=toggle_wgn)

    single_shot_switch = ft.Switch(label = SINGLE_SHOT_LABEL, on_change=toggle_single_shot)

    auto_run_switch = ft.Switch(label = AUTO_RUN_LABEL, on_change=toggle_auto_run)

    raw_requect_switch = ft.Switch(label = RAW_REQUEST_LABEL, on_change=toggle_raw_request)

    time_domain_switch = ft.Switch(label = TIME_DOMAIN_REQUEST_LABEL, on_change=toggle_time_d)
#######################################################
    config_table = ft.DataTable(
        border=ft.border.all(1, "black"),
        bgcolor = 'white',
        # width = 700,

            columns=[
                ft.DataColumn(ft.Text(CONTROL_PORT_STR, color = ft.colors.BLACK)),
                ft.DataColumn(ft.Text(DATA_PORT_STR,color = ft.colors.BLACK)),
                ft.DataColumn(ft.Text(TAPS_STR, color = ft.colors.BLACK)),
                ft.DataColumn(ft.Text(FREQ_RANGE_STR, color = ft.colors.BLACK)),
            ],
            rows=[
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(" ")),
                        ft.DataCell(ft.Text(" ")),
                        ft.DataCell(ft.Text(" ")),
                        ft.DataCell(ft.Text(" ")),
                    ],),
            ],
        )
    
    config_table2 = ft.DataTable(
        border=ft.border.all(1, "black"),
        bgcolor = 'white',
        column_spacing=0,

            columns=[
                ft.DataColumn(ft.Text(SETTING_STR, color = ft.colors.BLACK)),
                ft.DataColumn(ft.Text(STATUS_STR, color = ft.colors.BLACK)),
            ],
            rows=[
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(WGN_STR, color = ft.colors.BLACK)),
                        ft.DataCell(ft.Text(OFF_STR, color = ft.colors.RED)),
                    ],),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(SINGLE_SHOT_STR, color = ft.colors.BLACK)),
                        ft.DataCell(ft.Text(OFF_STR, color = ft.colors.RED)),
                        
                    ],),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(AUTO_RUN_STR, color = ft.colors.BLACK)),
                        ft.DataCell(ft.Text(OFF_STR, color = ft.colors.RED)),
                    ],),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(RAW_DATA_STR, color = ft.colors.BLACK)),
                        ft.DataCell(ft.Text(OFF_STR, color = ft.colors.RED)),
                    ],),
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(TD_STR, color = ft.colors.BLACK)),
                        ft.DataCell(ft.Text(OFF_STR, color = ft.colors.RED)),
                    ],),
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
                    ft.DataColumn(ft.Text(CONTROL_PORT_STR, color = ft.colors.BLACK)),
                    ft.DataColumn(ft.Text(DATA_PORT_STR, color = ft.colors.BLACK)),
                    ft.DataColumn(ft.Text(TAPS_STR, color = ft.colors.BLACK)),
                    ft.DataColumn(ft.Text(FREQ_RANGE_STR, color = ft.colors.BLACK)),
                ],
                rows=[
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(data_select.value, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(ctrl_select.value, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(tap_select.value, color = ft.colors.BLACK)) if tap_select.value else ft.DataCell(ft.Text(parametric_taps[0], color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(frequency_ranges[int(freq_range_select.value)], color = ft.colors.BLACK))
                                if freq_range_select.value
                                else ft.DataCell(ft.Text(frequency_ranges[0], color = ft.colors.BLACK))
                        ],),
                ],
            )

        updated_table2 = ft.DataTable(
            border=ft.border.all(1, "black"),
            bgcolor = 'white',
            column_spacing=0,
                columns=[
                    ft.DataColumn(ft.Text(SETTING_STR, color = ft.colors.BLACK)),
                    ft.DataColumn(ft.Text(STATUS_STR, color = ft.colors.BLACK)),
                    ],
                rows=[
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(WGN_STR, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(ON_STR if wgn_switch.value == 1 else OFF_STR, color = ft.colors.GREEN if wgn_switch.value == 1 else ft.colors.RED)),
                        ],),
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(SINGLE_SHOT_STR, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(ON_STR  if single_shot_switch.value == 1 else OFF_STR, color = ft.colors.GREEN if single_shot_switch.value == 1 else ft.colors.RED)),
                        ],),
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(AUTO_RUN_STR, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(ON_STR  if auto_run_switch.value == 1 else OFF_STR, color = ft.colors.GREEN if auto_run_switch.value == 1 else ft.colors.RED)),
                        ],),
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(RAW_DATA_STR, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(ON_STR if raw_requect_switch.value == 1 else OFF_STR, color = ft.colors.GREEN if raw_requect_switch.value == 1 else ft.colors.RED)),
                        ],),
                    ft.DataRow(
                        cells=[
                            ft.DataCell(ft.Text(TD_STR, color = ft.colors.BLACK)),
                            ft.DataCell(ft.Text(ON_STR  if time_domain_switch.value == 1 else OFF_STR, color = ft.colors.GREEN if time_domain_switch.value == 1 else ft.colors.RED)),
                        ],),
                ],
            )

        page.controls.clear()
        #page.add(ft.Column([Controls]),updated_table,chart)
        page.add(ft.Column([Controls]),updated_table,ft.Row([chart, updated_table2]))

        page.update()

    def select_data_port(e): #handle data port selection
        if os == MACOS_STR:
            portName = data_select.value.split("-")[0].strip()
            data_port.set(portName)
        elif os == LINUX_STR:
            portName = data_select.value.split("-")[0].strip()
            data_port.set(portName)
        elif os == WINDOWS_STR:
            portName = data_select.value
            data_port.set(portName)
        page.update()
    def select_ctrl_port(e): #handle ctrl port selection
        if os == MACOS_STR:
            portName = ctrl_select.value.split("-")[0].strip()
            ctrl_port.set(portName)
        elif os == LINUX_STR:
            portName = ctrl_select.value.split("-")[0].strip()
            ctrl_port.set(portName)
        elif os == WINDOWS_STR:
            portName = ctrl_select.value
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
    

    def select_tap_length(e):
        bytes_tmp = int(tap_select.value).to_bytes(2, 'little')
        temp_settings[7] = bytes_tmp[0]
        temp_settings[8] = bytes_tmp[1]
        print(temp_settings)
    def select_frange(e):
        tmp = int(freq_range_select.value)
        temp_settings[9] = tmp
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
        time_domain_switch


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
        #bgcolor= '#d5a6bd',
        bgcolor= BANNER_COLOR,
        actions=[
            ft.TextButton(ABOUT_US_TEXT, on_click = lambda _: page.go("/about")), 
            ft.IconButton(ft.icons.SETTINGS, on_click=open_modal),
            ft.IconButton(icon=ft.icons.UNDO)
            
        ]
    )

    #page.add(ft.Column([Controls]) ,chart ) 
    # page.add(ft.Column([Controls]),config_table,chart)
    page.add(ft.Column([Controls]),config_table,ft.Row([chart, config_table2]))
    
    
if __name__ == '__main__':
    ft.app(
        target=main,
        assets_dir='assets')
    
