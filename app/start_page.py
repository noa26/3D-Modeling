from backend import *
import styles
import tkinter as tk
import numpy as np

from matplotlib.patches import RegularPolygon
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class StartPage(tk.Frame):
    """
    The app's main frame.
    """

    def __init__(self, parent, controller, figure, ax):
        tk.Frame.__init__(self, parent, **styles.FRAME_STYLE)
        self.entries = dict()
        self.entries['file_type'] = tk.StringVar(self)

        # left side
        left_frame = tk.Frame(self, **styles.FRAME_STYLE)
        StartPage.add_canvas(left_frame, figure)
        tk.Button(left_frame, text="Update", **styles.BUTTON_STYLE,
                  command=lambda: StartPage.update_display(ax)).pack(side='bottom')
        left_frame.pack(side='left', expand=True)

        # right side - capture feature
        right_frame = tk.Frame(self, **styles.FRAME_STYLE)
        self.capture_feature(right_frame)
        right_frame.pack(side='right', expand=True)

        StartPage.update_display(ax)

    def capture_feature(self, frame):
        # capture button
        b_frame = tk.Frame(frame, **styles.COLORED_FRAME)
        capture_button_txt = tk.StringVar()
        capture_button_txt.set("Capture")
        capture_button = tk.Button(b_frame, **styles.BIG_BUTTON_STYLE, textvariable=capture_button_txt,
                                   command=lambda: capture(self.entries, capture_button_txt))
        capture_button.pack(expand=True)
        b_frame.pack(expand=True, pady=20, padx=20)

        # file name
        filename_entry = tk.Entry(frame, width=10, **styles.ENTRY_STYLE)
        filename_entry.insert(0, 'model')
        filename_entry.pack(side='left', padx=20)
        self.entries['file_name'] = filename_entry

        # file type
        options = ['.stl', '.xyz', '.obj', 'ply']
        self.entries['file_type'].set(options[0])
        file_type = tk.OptionMenu(frame, self.entries['file_type'], *options)
        file_type.config(**styles.MENU_STYLE, highlightthickness=0)
        file_type['menu'].config(**styles.MENU_STYLE)
        file_type.pack(side='left')

    @staticmethod
    def add_canvas(container, figure):
        canvas = FigureCanvasTkAgg(figure, container)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    @staticmethod
    def update_display(ax):
        ax.clear()

        h = RegularPolygon((0, 0), numVertices=6, radius=np.sqrt(1 / 3), alpha=0.2, facecolor=styles.WHITE)
        ax.add_patch(h)
        # h.radius = 0.5
        cameras = cams
        cameras = sorted(cameras, key=lambda c: c['angle'])
        vertices = [(a[0], a[1]) for a in h.get_path().vertices / 1.75]
        mid_points = [((vertices[i][0] + vertices[i + 1][0]) / 2, (vertices[i][1] + vertices[i + 1][1]) / 2) for i in
                      range(len(vertices) - 1)]

        for i in range(min(len(cameras), len(mid_points))):
            # add annotation
            ax.text(mid_points[i][0], mid_points[i][1], cameras[i]['serial'][-6:], color=styles.FG_COLOR, fontsize=8)
            ax.plot(mid_points[i][0], mid_points[i][1], 'o')

        ax.autoscale(enable=True)
        ax.set_aspect('equal')

    def calibrate(self):
        # get dist, height, width.

        pass
