import styles
import backend
import tkinter as tk
from typing import List

filters = ['decimation_filter', 'threshold_filter', 'disparity_transform', 'spatial_filter']
after_filters = ['temporal_filter', 'hole_filling_filter']


class ConfigurationWindow(tk.Toplevel):
    def __init__(self, parent):
        tk.Toplevel.__init__(self, **styles.WINDOW_STYLE)
        self.geometry("500x500")
        self.cameras = []
        canvas = tk.Canvas(self, **styles.CANVAS_STYLE)
        scroll_y = tk.Scrollbar(self, orient='vertical', command=canvas.yview)

        container = tk.Frame(canvas, **styles.FRAME_STYLE)

        f = tk.Frame(container, **styles.FRAME_STYLE)
        add_camera(f, self.cameras)
        f.pack()

        # Add Camera button
        tk.Button(container, text='Add Camera', **styles.BUTTON_STYLE,
                  command=lambda: add_camera(f, self.cameras)).pack()
        # Set Configurations button
        tk.Button(container, text='Set Configurations', ** styles.BUTTON_STYLE,
                  command=lambda: set_config(self.cameras)).pack()

        canvas.create_window(0, 0, anchor='nw', window=container)
        # make sure everything is displayed before configuring the scrollregion
        canvas.update_idletasks()

        canvas.configure(scrollregion=(0, 0, 5000, 5000),
                         yscrollcommand=scroll_y.set)

        canvas.pack(fill='both', expand=True, side='left')
        scroll_y.pack(fill='y', side='right')

        pass


def set_config(cameras: List[dict]):
    print("unimplemented set config", len(cameras))


def add_camera(parent: tk.Frame, cameras: List[dict]):
    subframe = tk.Frame(parent, **styles.FRAME_STYLE)
    cam_config = camera_form(subframe, 1)
    subframe.pack(side='top')

    header = 'Camera ' + str(len(cameras) + 1) + ": "
    tk.Label(subframe, text=header, **styles.LABEL2_STYLE).grid(row=0, column=0, pady=10, sticky='E')
    tk.Label(subframe, textvariable=cam_config['serial'], **styles.LABEL2_STYLE).grid(row=0, column=1, sticky='W')

    cameras.append(cam_config)
    pass


def camera_form(parent: tk.Frame, row=0):
    cam_config = {'serial': tk.StringVar(), 'distance': tk.DoubleVar(), 'angle': tk.IntVar(),
                  'filters': dict(), 'after_filters': dict()}

    # serial numbers
    row += 1
    tk.Label(parent, text='serial number: ', **styles.LABEL1_STYLE).grid(row=row, column=0, sticky='W', pady=10)
    serial_numbers = backend.available_cameras()
    sn = tk.OptionMenu(parent, cam_config['serial'], *serial_numbers)
    sn.config(**styles.MENU_STYLE, highlightthickness=0)
    sn['menu'].config(**styles.MENU_STYLE)
    sn.grid(row=row, column=1, sticky='W')
    cam_config['serial'].set('#' * 10)

    # distance
    row += 1
    tk.Label(parent, text='distance: ', **styles.LABEL1_STYLE).grid(row=row, column=0, sticky='W', pady=10)
    tk.Entry(parent, textvariable=cam_config['distance'], **styles.ENTRY_STYLE).grid(row=row, column=1, sticky='W')

    # angle
    row += 1
    tk.Label(parent, text='angle: ', **styles.LABEL1_STYLE).grid(row=row, column=0, sticky='W', pady=10)
    tk.Entry(parent, textvariable=cam_config['angle'], **styles.ENTRY_STYLE).grid(row=row, column=1, sticky='W')

    # filters
    cam_config['filters'], cam_config['after_filters'] = filters_form(parent, row)
    return cam_config


def filters_form(parent: tk.Frame, row=0):
    filters_d = {f: (tk.IntVar(), tk.StringVar()) for f in filters}
    after_filters_d = {f: (tk.IntVar(), tk.StringVar()) for f in after_filters}

    row += 1
    tk.Label(parent, text='filters', **styles.LABEL1_STYLE).grid()
    for filter_name, var in filters_d.items():
        row += 1
        tk.Checkbutton(parent, text=filter_name, variable=var[0], onvalue=1, offvalue=0,
                       command=lambda: print("hello :)"), **styles.CHECKBUTTON_STYLE).grid(sticky='W')
        tk.Entry(parent, textvariable=var[1], width=5, **styles.ENTRY_STYLE).grid(row=row, column=1)

    row += 1
    tk.Label(parent, text='after filters', **styles.LABEL1_STYLE).grid()
    for filter_name, var in after_filters_d.items():
        row += 1
        tk.Checkbutton(parent, text=filter_name, variable=var, onvalue=1, offvalue=0,
                       command=lambda: print("hello after :)"), **styles.CHECKBUTTON_STYLE).grid(sticky='W')
        tk.Entry(parent, textvariable=var[1], width=5, **styles.ENTRY_STYLE).grid(row=row, column=1)

    return filters_d, after_filters_d
