"""
GUI for handling and monitoring 3D Mapping.
"""

import styles
from backend import *
from start_page import StartPage
from about_page import AboutPage
from configuration_window import ConfigurationWindow
from amit.data import Data

import tkinter as tk
from tkinter import messagebox

import matplotlib
from matplotlib import style
from matplotlib import animation
from matplotlib.figure import Figure

matplotlib.use("TkAgg")
style.use("ggplot")

f = Figure(figsize=(3, 3), dpi=100, facecolor=styles.BG_COLOR)
ax = f.add_subplot(111)

ax.axes.get_xaxis().set_visible(False)
ax.axes.get_yaxis().set_visible(False)
ax.set_facecolor(styles.BG_COLOR)
for spine in ax.spines.values():
    spine.set_color(styles.BG_COLOR)


class DepthProcessingApp(tk.Tk):
    """
    Initialize the app's main components.
    """

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.status: int = 1
        self.frames: dict = dict()
        self.data: Data = Data.load_context(rs2.context())

        self.geometry("800x400")
        self.wm_title("3D Modeling")

        # menu bar
        menu_bar = self.create_menubar(self)
        self.config(menu=menu_bar)

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames[StartPage] = StartPage(container, self, figure=f, ax=ax)
        self.frames[AboutPage] = AboutPage(container, self)

        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.grid(row=0, column=0, sticky="nsew")
        frame.grid(row=0, column=0, sticky="nsew")
        frame.tkraise()

    def get_status(self):
        messagebox.showerror("New", "Unimplemented\n" + str(self.status))

    def create_menubar(self, container: tk.Tk):
        menu_bar = tk.Menu(container)

        # cameras cascade
        cameras_c = tk.Menu(menu_bar, tearoff=0)
        for camera in available_cameras():
            cameras_c.add_command(label="camera " + camera,
                                  command=lambda sn=camera: camera_display(sn))

        # configurations cascade
        config_c = tk.Menu(menu_bar, tearoff=0)
        config_c.add_command(label='edit configurations', command=lambda: self.edit_configurations())
        config_c.add_command(label='load configurations', command=lambda: self.load_configurations())
        config_c.add_command(label='save configurations', command=lambda: self.save_configurations())

        menu_bar.add_command(label='Start', command=lambda: self.show_frame(StartPage))
        menu_bar.add_cascade(label='Cameras', menu=cameras_c)
        menu_bar.add_command(label='Status', command=lambda: self.get_status())
        menu_bar.add_cascade(label='Configurations', menu=config_c)
        menu_bar.add_command(label='About', command=lambda: self.show_frame(AboutPage))

        return menu_bar

    def edit_configurations(self):
        ConfigurationWindow(self)

    def load_configurations(self):
        self.data = Data.load_json(open('config.json', 'r'))

    def save_configurations(self):
        Data.dump_json(self.data, open('config.json', 'w'))


if __name__ == "__main__":
    app = DepthProcessingApp()
    ani = animation.FuncAnimation(f, lambda *args: None, interval=200)
    app.mainloop()
