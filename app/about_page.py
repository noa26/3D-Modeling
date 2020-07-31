import styles
import tkinter as tk


class AboutPage(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent, **styles.FRAME_STYLE)

        header = tk.Label(self, text="About - Unimplemented", **styles.LABEL1_STYLE)
        header.pack()

    pass
