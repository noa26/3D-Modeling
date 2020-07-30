"""
GUI for handling and monitoring 3D Mapping.
"""

import styles
from util import *
from start_page import StartPage

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


def do_nothing(_):
	# TODO: load data from configuration file
	pass


class DepthProcessingApp(tk.Tk):
	"""
	Initialize the app's main components.
	"""

	def __init__(self, *args, **kwargs):
		tk.Tk.__init__(self, *args, **kwargs)

		self.status = 1
		self.geometry("800x400")
		self.wm_title("3D Modeling")
		self.frames = dict()

		# add menu-bar
		menu_bar = self.create_menubar(self)
		self.config(menu=menu_bar)

		container = tk.Frame(self)
		container.pack(side="top", fill="both", expand=True)
		container.grid_rowconfigure(0, weight=1)
		container.grid_columnconfigure(0, weight=1)

		self.frames[StartPage] = StartPage(container, self, figure=f, ax=ax)

		self.show_frame(StartPage)

	def show_frame(self, cont):
		frame = self.frames[cont]
		frame.grid(row=0, column=0, sticky="nsew")
		frame.grid(row=0, column=0, sticky="nsew")
		frame.tkraise()

	def get_status(self):
		messagebox.showerror("New", "Unimplemented\n" + str(self.status))

	def create_menubar(self, container):
		menu_bar = tk.Menu(container)

		# file cascade
		file_m = tk.Menu(menu_bar, tearoff=0)
		file_m.add_command(label="status", command=lambda: self.get_status())
		menu_bar.add_cascade(label="File", menu=file_m)

		# cameras cascade
		cameras_f = tk.Menu(menu_bar, tearoff=0)
		for camera in connected_cameras():
			cameras_f.add_command(label="camera " + str(camera), command=lambda: camera_display(camera))
		menu_bar.add_cascade(label="Cameras", menu=cameras_f)

		menu_bar.add_command(label="About", command=lambda: print("unimplemented\tabout command"))

		return menu_bar


if __name__ == "__main__":
	app = DepthProcessingApp()
	ani = animation.FuncAnimation(f, do_nothing, interval=200)
	app.mainloop()
