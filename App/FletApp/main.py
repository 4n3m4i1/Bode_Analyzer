
import matplotlib.axes
import matplotlib.axis
import matplotlib.figure
import matplotlib.lines
import webbrowser
from generic_include import *
from generic_include import BANDIT_SETTINGS_BYTES
from assets.ref import *
import serial
import webbrowser
# from enum import IntEnum
# import serial.tools.list_ports_osx as list_ports_osx
# import serial.tools.list_ports_windows as list_ports_windows
# import serial.tools.list_ports_linux as list_ports_linux
import serial.tools.list_ports as list_ports
import platform
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from itertools import chain
import struct 
import flet as ft
from flet.matplotlib_chart import MatplotlibChart
from flet import RouteChangeEvent
from multiprocess import Queue, Event
import multiprocessing
from multiprocessing import Queue, Lock
from threading import Thread
import time
from bitstring import BitArray
import numpy as np
os = platform.system()
negative_allowed_event = Event()
negative_allowed_event.clear()
start_event = Event()
queue_max = Event()
queue_max.clear()
settings_event = Event()

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
    print(port)
    port_selections.append(ft.dropdown.Option(str(port)))

taps_list =[]
size = len(parametric_taps)
for index in range(size):
    taps_list.append(ft.dropdown.Option(key=parametric_taps[index]))


range_list = []
size1 = len(frequency_ranges)
for index1 in range(size1):
    range_list.append(ft.dropdown.Option(key=index1, text=frequency_ranges[index1]))

data_port = Port()
ctrl_port = Port()
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
            skip_lmsset = newlearnin_rate = BitArray(hex=CTRLCHANNEL.read(1).hex())
            skip_fftset = newlearnin_rate = BitArray(hex=CTRLCHANNEL.read(1).hex())
            print(setting)
            print(bandit_sett_bf)
            print(newfreqrange)
            print(newtaplen)
            print(new_err_limit)
            print(new_lms_offset)
            print(newlmsmax_attempts)
            print(newlearnin_rate)
            if(skip_fftset): 
                print("Skip FFT")
            if(skip_lmsset): 
                print("Skip LMS")
            # print(thing.bin)
            # CTRLCHANNEL.flush()
            while True:
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
                        skip_lmsset = newlearnin_rate = BitArray(hex=CTRLCHANNEL.read(1).hex())
                        skip_fftset = newlearnin_rate = BitArray(hex=CTRLCHANNEL.read(1).hex())
                        print(setting)
                        print(bandit_sett_bf)
                        print(newfreqrange)
                        print(newtaplen)
                        print(new_err_limit)
                        print(new_lms_offset)
                        print(newlmsmax_attempts)
                        print(newlearnin_rate)
                        if(skip_fftset): 
                            print("Skip FFT")
                        if(skip_lmsset): 
                            print("Skip LMS")
                        settings_event.clear()
                else:
                    if DATACHANNEL.in_waiting >= CDC_PACKET_LENGTH:
                        start = time.time()
                        header_data = DATACHANNEL.read(CDC_PACKET_LENGTH)
                        count = 0
                        CTRLCHANNEL.write(b'a')
                        num_samples = (header_data[3] * 256 + header_data[2]) * BYTES_PER_NUMBER
                        while DATACHANNEL.in_waiting < num_samples and count < 1000:
                            count = count + 1
                        f_data = DATACHANNEL.read(num_samples)
                        end = time.time()
                        print(f'{end - start} second transmition')
                        lock.acquire()
                        if not queue_max.is_set(): #if the queue is not full, obtain new data: this will help alleviate graph delays
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
    while True:
        is_set = GraphEvent.wait()
        try:
            graph_data_raw = data_in_Queue.get(block=False)
            unpacked_graph_data = struct.iter_unpack('h', graph_data_raw)
            listed_graph_data = list(chain.from_iterable(unpacked_graph_data))
            converted_graph_data = Q15_to_float_array(listed_graph_data, len(listed_graph_data))
            data_out_Queue.put(converted_graph_data, block=False)
        except Exception as e:
            pass
##########################################################################################UPDATEGRAPH
def update_graph(data_Queue: Queue, chart: MatplotlibChart, line: matplotlib.lines.Line2D, frange_queue: Queue, axis, fig):
    frange = 0
    print('start graph')
    while True:
        #Checks if the data queue is not full
        print(queue_max.is_set())
        if data_Queue.qsize() >= MAX_QUEUE_SIZE:
            queue_max.set()
        elif data_Queue.qsize() < MIN_QUEUE_SIZE and queue_max.is_set():
            queue_max.clear()

        is_set = GraphEvent.wait()
        try:
            data = data_Queue.get()
            line.set_data(np.arange(len(data)), data)
            if frange_queue.empty():
                if frange == 0:
                    frange = 250e3
                    # ticks_x = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(frange / x))
                    axis.xaxis.set_major_locator(ticker.LinearLocator(11))
                    axis.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format((x / (len(data) - 1)) * frange)))
                    axis.xaxis.set_minor_locator(ticker.NullLocator())
                    axis.xaxis.set_minor_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format((x / (len(data) - 1)) * frange)))
                    axis.grid(True)
                else:
                    pass
            else:

                frange = frange_queue.get()
                
                axis.xaxis.set_major_locator(ticker.LinearLocator(11))
                axis.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format((x / (len(data) - 1)) * frange)))
                axis.xaxis.set_minor_locator(ticker.NullLocator())
                axis.xaxis.set_minor_formatter(ticker.FuncFormatter(lambda x, pos: '{0:g}'.format((x / (len(data) - 1)) * frange)))
                axis.grid(True)
            if data:
                if max(data) != 0:
                    print("Valid Data Print!!")
                    plt.xlim(0, len(data) - 1)
                    
                    #plt.ylim(min(data), max(data) + 1e-13)
                    if negative_allowed_event.is_set():
                        plt.ylim(min(data)*1.1, max(data)*1.1)
                        #plt.ylim(min(data[1:len(data)-1])*1.1, max(data[1:len(data)-1])*1.1)
                    else:
                        #plt.ylim(0, max(data[1:len(data)-1])*1.1)
                        plt.ylim(min(data)*1.1, max(data)*1.1)
                        #plt.ylim(0, max(data)*1.1)
                        #plt.ylim(-0.005, 0.1)
                    axis.draw_artist(line)
                    print('oop')
                    fig.canvas.blit(fig.bbox)  # I added this
                    fig.canvas.flush_events()
                    print('here')
                    chart.update()
                    print('huh')
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

    page.window_width = 1000     
    #page.window_height = 1000  

    #page.window_resizable = False
    #page.window_full_screen = True

    page.scroll=ft.ScrollMode.ALWAYS## test
    page.theme = ft.Theme(
    color_scheme=ft.ColorScheme(
        primary=ft.colors.BLACK,   
    )
)
    temp_settings = INIT_SETTINGS

    def open_youtube(e):
        webbrowser.open(YOUTUBE_URL)


    pool = multiprocessing.Pool(processes=1)
    m = multiprocessing.Manager()
    FFT_real_queue = m.Queue()
    FFT_converted_queue = m.Queue()
    Settings_Queue = m.Queue()
    FRANGE_queue = m.Queue()
    lock = Lock()
    temp_settings = INIT_SETTINGS


    #video
    def open_youtube(e):
        youtube_url = "https://youtu.be/heG2l5tyS7A?si=Tvam4DrzBTFw0_lb" 
        webbrowser.open(youtube_url)

    #LINKDIN
    def lance_contact(e):
        lance_url = "https://www.linkedin.com/in/lance-m-reyes/" 
        webbrowser.open(lance_url)

    def joseph_contact(e):
        joseph_url = "https://www.linkedin.com/in/joseph-adrian-de-vico/"
        webbrowser.open(joseph_url)

    def ari_contact(e):
        ari_url = "https://www.linkedin.com/in/arianna-sarahi-bergado-84863b272/" 
        webbrowser.open(ari_url)
    
    def josh_contact(e):
        josh_url = "https://www.linkedin.com/in/joshkylecole/?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app"
        webbrowser.open(josh_url)
    
    def christian_contact(e):
        christian_url = "https://www.linkedin.com/in/christian-abellacompe/"
        webbrowser.open(christian_url)
    
    # def image_change(page:ft.Page):
    logo = ft.Container(
        ft.Image(
        src= BANDIT_LOGO_SRC,
        width=130,
        height=130,
        fit=ft.ImageFit.CONTAIN,),
        alignment=ft.alignment.center,
    )
            
    diagram=ft.Container(
        ft.Image(
        src= DIAGRAM,
        width=600,
        height=600,
        fit=ft.ImageFit.CONTAIN,),
        alignment=ft.alignment.center,
    )
    
    c = ft.AnimatedSwitcher(
        logo,
        transition=ft.AnimatedSwitcherTransition.SCALE,
        duration=500,
        reverse_duration=100,
        switch_in_curve=ft.AnimationCurve.BOUNCE_OUT,
        switch_out_curve=ft.AnimationCurve.BOUNCE_IN,
    )

    def animate(e):
        c.content = diagram if c.content == logo else logo
        page.update()

        # return c, ft.ElevatedButton("Click", on_click=animate)
       

    

    
    #HARD CODED KEY SPECIFICATIONS 
        #SPEC ONE 
    def panel_one(page: ft.Page):
        def handle_change(e: ft.ControlEvent):
            print(f"change on panel with index {e.data}")
            #print("Event type:", e.type)
            page.update()

        panel = ft.ExpansionPanelList(
            expand_icon_color= DARK_PINK,
            elevation=8,
            divider_color=DARK_PINK,
            on_change=handle_change,
            expanded_header_padding=ft.Padding(top=25, left=20, right=20, bottom=0),
            controls=[
                ft.ExpansionPanel(
                    header=ft.Text("USB", theme_style=ft.TextThemeStyle.TITLE_MEDIUM, text_align=ft.TextAlign.CENTER ),
                    content = ft.Text("-Powered: Operates across entire USB 2.0 voltage range\n-High speed USB Communications Device Class (CDC) interface\n ",theme_style=ft.TextThemeStyle.BODY_MEDIUM, text_align=ft.TextAlign.CENTER),
                    bgcolor=LIGHT_PINK,
                    expanded=False,
                    
                ),
                ft.ExpansionPanel(
                    header=ft.Text("Input / Ouput", theme_style=ft.TextThemeStyle.TITLE_MEDIUM, text_align=ft.TextAlign.CENTER ),
                    content = ft.Text("-Direct from output of GWN fed SUT (4Vpk-pk Signal max)\n-Gaussian White Noise (GWN), input for System Under Test\n",theme_style=ft.TextThemeStyle.BODY_MEDIUM, text_align=ft.TextAlign.CENTER),
                    bgcolor=LIGHT_PINK,
                    expanded=False,
                ),
                ft.ExpansionPanel(
                    header=ft.Text("Digital Signal Processing", theme_style=ft.TextThemeStyle.TITLE_MEDIUM, text_align=ft.TextAlign.CENTER ),
                    content = ft.Text("-Dynamic Downsampling\n-Least Mean Squared (LMS) Adaptive Filter\n-Cooley-Tukey FFT\nFully Q15 fixed point routines. Optimized for dual-core ARM M0+\n",theme_style=ft.TextThemeStyle.BODY_MEDIUM, text_align=ft.TextAlign.CENTER),
                    bgcolor=LIGHT_PINK,
                    expanded=False,
                ),
                ft.ExpansionPanel(
                    header=ft.Text("Graphical User Interface", theme_style=ft.TextThemeStyle.TITLE_MEDIUM, text_align=ft.TextAlign.CENTER ),
                    content = ft.Text("-Multiprocessing cross-platform Python script\n",theme_style=ft.TextThemeStyle.BODY_MEDIUM, text_align=ft.TextAlign.CENTER),
                    bgcolor=LIGHT_PINK,
                    expanded=False,
                ),
                ft.ExpansionPanel(
                    header=ft.Text("Analysis Ranges", theme_style=ft.TextThemeStyle.TITLE_MEDIUM, text_align=ft.TextAlign.CENTER ),
                    content = ft.Text("- 20Hz-31kHz\n-20Hz-62kHz\n-20Hz-125kHz\n",theme_style=ft.TextThemeStyle.BODY_MEDIUM, text_align=ft.TextAlign.CENTER),
                    bgcolor=LIGHT_PINK,
                    expanded=False,
                )
            ]
        )
        return panel
    
    # def grid(page: ft.Page)
    #     def handle_change(e: ft.ControlEvent):
    #         print(f"change on panel with index {e.data}")
    #         #print("Event type:", e.type)
    #         page.update()

    #         images = ft.GridView(
    #             expand=1
    #             runs_count=5
    #             max_extent = 150,
    #             child_aspect_ratio=1.0,
    #             spacing=5,
    #             run_spacing=5,
    #         )

        


    #ABOUT US MAIN
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

            picture_switch = ft.ElevatedButton(
                text = "Diagram", 
                on_click=animate,
            )

            key_specs = panel_one(e.page)
            #switch_image = image_change(e.page)
            
            button_row = ft.Row(
                controls=[
                    ft.ElevatedButton("Lance Reyes", on_click=lambda _:page.go("/lance")),  #meet_ari #team_pictures, meet_ari,
                    ft.ElevatedButton("Joseph De Vico", on_click=lambda _:page.go("/joseph")),
                    ft.ElevatedButton("Arianna-Sarahi Bergado", on_click=lambda _:page.go("/ari")),
                    ft.ElevatedButton("Joshua Cole", on_click=lambda _:page.go("/josh")),
                    ft.ElevatedButton("Christian Abella", on_click=lambda _:page.go("/christian")),

                ],
                spacing=10,
                alignment=ft.MainAxisAlignment.CENTER
            )

            # ari_collage = ft.Row(
            #     controls = [
            #         ft.Image(src= ARI_IMAGE,width=140,height=140,fit=ft.ImageFit.CONTAIN,),
            #         ft.Image(src= ARI_HOBBY, width=140,height=140,fit=ft.ImageFit.CONTAIN,),
            #     ],
            #     spacing=10,
            #     alignment=ft.MainAxisAlignment.CENTER
            # )
            
            controls_about = [
                ft.AppBar(title=ft.Text(ABOUT_TITLE), bgcolor=BANNER_COLOR, actions=[picture_switch, youtube_button]),
                #switch_image
                #logo, 
                c, overview_title, overview_statement, specs_title, key_specs, 
                #specs_description,technologies_title, technologies_description,
                meet_team, team_description, button_row
                
            ] 
            alignment=ft.MainAxisAlignment.CENTER,
            
            page.views.append(
                ft.View(
                    route="/about",
                    controls = controls_about 
                )
            )


#######LANCE ABOUT US
        if page.route == "/lance":
            page.vertical_alignment = ft.MainAxisAlignment.CENTER
            page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
            page.theme = ft.Theme(
                color_scheme_seed=LIGHT_PINK,
            )
            lance_button =ft.ElevatedButton(
            text="Connect with Me",
            on_click=lance_contact,
            icon=ft.icons.FAVORITE
            )
        #if page.route == "/lance":
            page.views.append(
                ft.View(
                    "/lance",
                    [
                        ft.AppBar(title=ft.Text("Bode Bandits"), bgcolor=BANNER_COLOR, actions=[lance_button]),
                        #ft.ElevatedButton("Exit", on_click=lambda _: page.go("/about")),
                        lance_title, lance_collage, lance_description,
                    ]
                )
            )

#######JOSEPH ABOUT US
        
        if page.route == "/joseph":
            page.vertical_alignment = ft.MainAxisAlignment.CENTER
            page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
            page.theme = ft.Theme(
                color_scheme_seed=LIGHT_PINK,
            )
            joseph_button =ft.ElevatedButton(
            text="Connect with Me",
            on_click=joseph_contact,
            icon=ft.icons.FAVORITE
            )
        #if page.route == "/joseph":
            page.views.append(
                ft.View(
                    "/joseph",
                    [
                        ft.AppBar(title=ft.Text("Bode Bandit"), bgcolor=BANNER_COLOR, actions=[joseph_button]),
                        #ft.ElevatedButton("Exit", on_click=lambda _: page.go("/about")),
                        joseph_title, joseph_collage, joseph_description,
                    ]
                )
            )

#######ARI ABOUT US
        if page.route == "/ari":
            page.vertical_alignment = ft.MainAxisAlignment.CENTER
            page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
            page.theme = ft.Theme(
                color_scheme_seed=LIGHT_PINK,
            )
            ari_button =ft.ElevatedButton(
            text="Connect with Me",
            on_click=ari_contact,
            icon=ft.icons.FAVORITE
            )
        #if page.route == "/ari":
            page.views.append(
                ft.View(
                    "/ari",
                    [
                        ft.AppBar(title=ft.Text("Bode Bandits"), bgcolor=BANNER_COLOR, actions=[ari_button]),
                        #ft.ElevatedButton("Exit", on_click=lambda _: page.go("/about")),
                        ari_title, ari_onecollage, ari_description,
                    ]
                )
            )



#######JOSH ABOUT US
        if page.route == "/josh":
            page.vertical_alignment = ft.MainAxisAlignment.CENTER
            page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
            page.theme = ft.Theme(
                color_scheme_seed=LIGHT_PINK,
            )
            josh_button =ft.ElevatedButton(
            text="Connect with Me",
            on_click=josh_contact,
            icon=ft.icons.FAVORITE
            )
        #if page.route == "/josh":
            page.views.append(
                ft.View(
                    "/josh",
                    [
                        ft.AppBar(title=ft.Text("Bode Bandits"), bgcolor=BANNER_COLOR, actions=[josh_button]),
                        #ft.ElevatedButton("Exit", on_click=lambda _: page.go("/about")),
                        josh_title, josh_collage, josh_description,
                    ]
                )
            )



#######CHRISTIAN ABOUT US
        if page.route == "/christian":
            page.vertical_alignment = ft.MainAxisAlignment.CENTER
            page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
            page.theme = ft.Theme(
                color_scheme_seed=LIGHT_PINK,
            )
            christian_button =ft.ElevatedButton(
            text="Connect with Me",
            on_click=christian_contact,
            icon=ft.icons.FAVORITE
            )
        #if page.route == "/christian":
            page.views.append(
                ft.View(
                    "/christian",
                    [
                        ft.AppBar(title=ft.Text("Bode Bandits"), bgcolor=BANNER_COLOR, actions=[christian_button]),
                        #ft.ElevatedButton("Exit", on_click=lambda _: page.go("/about")),
                        christian_title, christian_collage, christian_description,
                        
                    ]
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
        if start_event.is_set() and GraphEvent.is_set():
            start_event.clear()
            GraphEvent.clear()
            stop_settings = temp_settings
            stop_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTORUN.value] = 0x00
            stop_settings[BANDIT_SETTINGS_BYTES.USBBSRX_AUTOSEND.value] = 0x00
            Settings_Queue.put(stop_settings)

            settings_event.set()
        else:
            pass

    
        
    ## Graph configurations
    figure = plt.figure(figsize=(15,7))
    ax = figure.add_subplot(111)
    line, = ax.plot(init_graph, animated=True,)
    ax.grid()

    line.set_color('#F55BB0')
    chart = MatplotlibChart(figure, expand=True,)
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
    
    
    ## CONTAINERS for the about us page
    # logo = ft.Container(
    #     content = ft.Image(
    #         src= BANDIT_LOGO_SRC,
    #         width=140,
    #         height=140,
    #         fit=ft.ImageFit.CONTAIN,
    #     ),
    #     alignment = ft.alignment.center,
    # )
    
        
    overview_title = ft.Container(
        content = ft.Text("Project Overview", theme_style=ft.TextThemeStyle.TITLE_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )

    overview_statement = ft.Container(
        ft.Text("Bode Analysis N’ Display of Instrument Testing (BANDIT) is a small, portable, and inexpensive Frequency Response Analyzer aimed at providing rapid assessments of a system’s transfer function. User selectable resolutions and analysis ranges, as well as output visualization, are provided by a comprehensive Graphical User Interface. BANDIT is capable of in depth self-calibration routines correcting for board to board component tolerances and voltage supply fluctuations, ensuring performance across devices without expensive component binning. Double click the button labeled Diagram to take a internal look at the BANDIT.", theme_style=ft.TextThemeStyle.BODY_LARGE),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
    specs_title =  ft.Container(
        ft.Text("Key Specifications", theme_style=ft.TextThemeStyle.TITLE_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
        
    

    meet_team =  ft.Container(
        ft.Text("Meet the Bode Bandits", theme_style=ft.TextThemeStyle.TITLE_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
    
    team_description =  ft.Container(
        ft.Text("The Bode Bandits, composed of five Computer and Electrical Engineering students: Christian Abella, Joshua Cole, Lance Reyes, Joseph De Vico, and Arianna-Sarahi Bergado. We have been tasked with an Electrical/Computer Engineering (ECE) Project. Double click each name below to learn more about us.", theme_style=ft.TextThemeStyle.BODY_LARGE, text_align= ft.TextAlign.CENTER),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    


#########LANCE PAGE 
    lance_title = ft.Container(
        content = ft.Text("Lance Reyes", theme_style=ft.TextThemeStyle.DISPLAY_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )
    
    lance_collage = ft.Container(
        content=ft.Row(
            controls = [
                ft.Image(src= LANCE_IMAGE,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= LANCE_FRIENDS, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= LANCE_HOBBY,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= LANCE_FAM, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
            ],
            spacing=10,
            alignment=ft.MainAxisAlignment.CENTER
        )
    )


    lance_description = ft.Container(
        ft.Text("Hi, I’m Lance Reyes. I was born and raised in San Diego and have lived here my whole life. I am majoring in computer engineering and will graduate in the spring of 2024. I got into engineering because I’ve always had an interest in building and putting things together. As well as tinkering with items and understanding how they work. Ultimately, I enjoy coding and the flexibility of the software side which allows me to build projects of any scale. My role in building the BANDIT was mainly designing the GUI for the BANDIT to communicate to but my other responsibilities included soldering components and assisting with USB protocol and data streaming. Outside of engineering I enjoy watching the San Diego Padres, kicking back and having a good time with some friends, or watching the latest movies recommended to me.",
                 theme_style=ft.TextThemeStyle.HEADLINE_MEDIUM, text_align= ft.TextAlign.CENTER),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    

#########JOSEPH PAGE 
    joseph_title = ft.Container(
        content = ft.Text("Joseph De Vico", theme_style=ft.TextThemeStyle.DISPLAY_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )
    
    joseph_collage = ft.Container(
        content=ft.Row(
            controls = [
                ft.Image(src= JOSEPH_IMAGE,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= JOSEPH_FRIENDS, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= JOSEPH_HOBBY,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= JOSEPH_SOLO, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
            ],
            spacing=10,
            alignment=ft.MainAxisAlignment.CENTER
        )
    )


    joseph_description = ft.Container(
        ft.Text("I'm Joseph, a former machinist and fabricator finishing out my Computer Engineering undergraduate degree at San Diego State University. I’ve long been engaging in hobby electronics, initially drawn in by analog synthesizers. I’m predominantly interested in digital signal processing, mixed signal design, and systems engineering. Within the project I helped implement out DSP algorithms, executed the digital design and systems planning for our board, and laid the groundwork for architecture specific optimizations for the BANDIT. Moving forward, I’d like to work in modem and phased array communications system design. If you or a friend has a good connection at Qualcomm please get in touch! :)",
                 theme_style=ft.TextThemeStyle.HEADLINE_MEDIUM, text_align= ft.TextAlign.CENTER),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    


#########ARI PAGE
    ari_title = ft.Container(
        content = ft.Text("Arianna-Sarahi Bergado", theme_style=ft.TextThemeStyle.DISPLAY_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )
    
    ari_onecollage = ft.Container(
        content=ft.Row(
            controls = [
                ft.Image(src= ARI_IMAGE,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= ARI_HOBBY, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= ARI_FAM,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= ARI_GRAD, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
            ],
            spacing=10,
            alignment=ft.MainAxisAlignment.CENTER
        )
    )


    ari_description = ft.Container(
        ft.Text("Hello! My name is Arianna-Sarahi Bergado. Born and raised in Chula Vista, I’ve called San Diego home all my life. As the youngest of four sisters, family has always been a big part of my life. In my leisure time, I enjoy staying active through weightlifting and hiking, and I cherish the moments spent with family and friends. I am currently pursuing a degree in Computer Engineering, with an anticipated graduation in Spring 2024, My academic interests are diverse, spanning Web Development to Analog and Digital Communications. My role in this project includes designing the Graphical User Interface(GUI), soldering printed circuit boards (PCBs), and being team lead. As graduation approaches, I am eager and excited for the new opportunities that await in my professional journey.",
                 theme_style=ft.TextThemeStyle.HEADLINE_MEDIUM, text_align= ft.TextAlign.CENTER),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )

##########JOSH PAGE
    josh_title = ft.Container(
        content = ft.Text("Joshua Cole", theme_style=ft.TextThemeStyle.DISPLAY_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )
    josh_collage = ft.Container(
        content=ft.Row(
            controls = [
                ft.Image(src= JOSH_IMAGE,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= JOSH_FAM, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= JOSH_HOBBY,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= JOSH_FRIENDS, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
            ],
            spacing=10,
            alignment=ft.MainAxisAlignment.CENTER
        )
    )

    josh_description = ft.Container(
        ft.Text("Hello, my name is Joshua Cole. I am a Navy veteran and will be graduating with a Bachelor's degree in Electrical Engineering in Spring 2024. I hail from Minnesota and am currently undertaking an internship at the Naval Information Warfare Center (NIWC), where I collaborate with an interdisciplinary team on advancing research projects aimed at enhancing warfighter capabilities.In the realm of electrical engineering, my areas of interest include radio frequency engineering and electrical design. Throughout my recent project, I engaged in PCB creation, conducted MATLAB system overviews, designed filters, and developed firmware.In addition to my academic and professional pursuits, I am passionate about trail running, hiking, and weight lifting. With graduation on the horizon, I am eager to devote more time to these hobbies.",
                 theme_style=ft.TextThemeStyle.HEADLINE_MEDIUM, text_align= ft.TextAlign.CENTER),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )
    
#########CHRISTIAN PAGE
    christian_title = ft.Container(
        content = ft.Text("Christian Abella", theme_style=ft.TextThemeStyle.DISPLAY_LARGE, weight=ft.FontWeight.BOLD, text_align= ft.TextAlign.CENTER),
        bgcolor=DARK_PINK,
        border=ft.border.all(2, LIGHT_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        
        )
    christian_collage = ft.Container(
        content=ft.Row(
            controls = [
                ft.Image(src= CHRISTIAN_IMAGE,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= CHRISTIAN_HOBBY, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= CHRISTIAN_FRIENDS,width=350,height=350,fit=ft.ImageFit.CONTAIN,),
                ft.Image(src= CHRISTIAN_GRAD, width=350,height=350,fit=ft.ImageFit.CONTAIN,),
            ],
            spacing=10,
            alignment=ft.MainAxisAlignment.CENTER
        )
    )

    christian_description = ft.Container(
        ft.Text("Hello, my name is Christian Abella. I am from Bay Area, CA. I am passionate about cooking, music, being in nature, and hanging out with my friends! I wanted to become an engineer because I want to build devices and software that help people all over the world. I am a computer engineering major and I am most interested in software engineering and embedded systems. I was on the Graphical User Interface (GUI) team where I designed the front end of our application and engineered the multithreaded backend. I have the privilege of working as a Software Engineer at Atlassian starting in the summer! I look forward to applying all that I learned through this project onto my future career.",
                 theme_style=ft.TextThemeStyle.HEADLINE_MEDIUM, text_align= ft.TextAlign.CENTER),
        bgcolor=LIGHT_PINK,
        border=ft.border.all(2, DARK_PINK),
        padding = ft.padding.all,
        width=2000,
        alignment = ft.alignment.center,
        )

    # meet_ari = ft.Container(
    #     ft.ElevatedButton("Arianna-Sarahi Bergado", on_click=lambda _: page.go("/ari")),
    #     ft.ElevatedButton("Exit", on_click=lambda _:page.go("/about"))
    # )

#   specs_description = ft.Container(
    #     ft.DataTable(
    #         width=2000,
    #         bgcolor = LIGHT_PINK,
    #         border = ft.border.all(2,DARK_PINK),
    #         #border_radius = 10,
    #         vertical_lines=ft.border.BorderSide(width=2, color=DARK_PINK),
    #         horizontal_lines=ft.border.BorderSide(width=2, color=DARK_PINK),
    #         sort_column_index=0,
    #         sort_ascending=True,
    #         heading_row_color=ft.colors.BLACK12,
    #         heading_row_height=0,
    #         data_row_max_height=25,
    #         data_row_color= LIGHT_PINK,
    #         column_spacing= 200,
    #         columns = [
    #             ft.DataColumn(ft.Text("")),
    #             ft.DataColumn(ft.Text(""),numeric=False),

    #         ],
    #         rows=[
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("USB Powered")),
    #                      ft.DataCell(ft.Text("Operates across entire USB 2.0 voltage range")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("Input")),
    #                     ft.DataCell(ft.Text("Direct from output of GWN fed SUT (4Vpk-pk Signal max)")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("Output:")),
    #                      ft.DataCell(ft.Text("Gaussian White Noise (GWN), input for System Under Test")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("Digital Signal Processing")),
    #                      ft.DataCell(ft.Text("Dynamic Downsampling, Least Mean Squared (LMS) Adaptive Filter, Cooley-Tukey FFT, Fully Q15 fixed point routines. Optimized for dual-core ARM M0+")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("Greater than 1 HZ update Rate")),
    #                      ft.DataCell(ft.Text("")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("GUI")),
    #                      ft.DataCell(ft.Text("Multiprocessing cross-platform Python script")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("High speed USB Communications Device Class (CDC) interface")),
    #                      ft.DataCell(ft.Text("")),
    #                 ],
    #             ),
    #             ft.DataRow(
    #                 cells=[
    #                     ft.DataCell(ft.Text("Analysis Ranges")),
    #                      ft.DataCell(ft.Text("20Hz - 31kHz, 20Hz - 62kHz, 20Hz - 125kHz")),                
    #                 ],
    #     #        ),
    #    #     ],
    #   #  ),
    #  #   alignment=ft.alignment.center
    # #)

 
    
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
        bgcolor='#c27ba0',
        border_radius=20,
    )

    # change bit at index 3
    def toggle_wgn(e):
        if wgn_switch.label == WGN_LABEL_ON:
            temp_settings[BANDIT_SETTINGS_BYTES.USBBSRX_WGN_ALWAYS_ON.value] = 0
        else:
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
        negative_allowed_event.set() if not negative_allowed_event.is_set() else negative_allowed_event.clear()
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

    def reset_modal(e):
  
        page.update()
    
    def exit_modal(e):
        SettingsSelection.open = False
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
        elif os == WINDOWS_STR:
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
        elif os == WINDOWS_STR:
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

        dividy = MAX_TAPS / taplen

        bytes_tmp = bytes_tmp / dividy

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
    learning_rate_text = ft.Text(value=LEARNING_RATE_LABEL)
    learning_rate_slider = ft.Slider(
        label = '{value}',
        min=0x01, 
        max=0x7FFF, 
        # min = float(0.0),
        # max = float(1.0),
        active_color=STOP_BUTTON_COLOR,
        inactive_color=START_BUTTON_COLOR,
        divisions=32767, 
        on_change=change_LR
        )
    offset_text = ft.Text(value=OFFSET_LABEL)
    offset_delay_slider = ft.Slider(
        label = "{value}",
        min = -MAX_TAPS/2, 
        max = MAX_TAPS/2, 
        active_color=STOP_BUTTON_COLOR,
        inactive_color=START_BUTTON_COLOR,
        divisions = MAX_TAPS, 
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
        learning_rate_text,
        learning_rate_slider,
        offset_text,
        offset_delay_slider
        ],
        scroll=ft.ScrollMode.ALWAYS

    )

    SettingsSelection = ft.AlertDialog(
        #modal=True,
        modal=False,
        #title=ft.Text(CONFIG_STR),
        title = ft.Row([ft.Text(CONFIG_STR),ft.IconButton(ft.icons.CLOSE ,on_click= exit_modal)], spacing = 450),
        content= ConfigDisplay,
        #actions=[ft.TextButton(CLOSE_STR, on_click=close_modal)],
        actions=[ft.TextButton(RESET_STR, on_click=reset_modal) , ft.TextButton(CLOSE_STR, on_click=close_modal)],
        actions_alignment=ft.MainAxisAlignment.END,
    )
    page.appbar = ft.AppBar(
        leading=ft.IconButton(ft.icons.BREAKFAST_DINING_OUTLINED),

        title=ft.Text(APPBAR_TITLE),
        bgcolor= '#d5a6bd',
        actions=[
            ft.TextButton(ABOUT_US_TEXT, on_click = lambda _: page.go("/about")), 
            ft.IconButton(ft.icons.SETTINGS, on_click=open_modal),
            ft.IconButton(icon=ft.icons.UNDO)
            
        ]
    )

    #page.add(ft.Column([Controls]) ,chart ) 
    page.add(ft.Column([Controls]),config_table,chart)

    serial_reader = Thread(target=serial_read, args=(data_port, ctrl_port, FFT_real_queue, Settings_Queue, page, lock))
    serial_reader.daemon = True
    data_converter_process = Thread(target=raw_data_to_float_converter, args=(FFT_converted_queue, FFT_real_queue, lock))
    data_converter_process.daemon = True
    update_graph_thread = Thread(target=update_graph, args=(FFT_converted_queue, chart, line, FRANGE_queue, ax, figure))
    update_graph_thread.daemon = True
    update_graph_thread.start()
    serial_reader.start()
    data_converter_process.start()
    
if __name__ == '__main__':
    ft.app(
        target=main,
        assets_dir='assets')