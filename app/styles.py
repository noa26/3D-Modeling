GREY = "#201f24"
BLUE = "#87CEEB"
WHITE = "#dbdadf"
RED = "#FF6347"

BG_COLOR = GREY
FG_COLOR = BLUE

LARGE_FONT = ("Bahnschrift light", 25)
SMALL_FONT = ("Bahnschrift light", 10)

FRAME_STYLE = {'background': BG_COLOR}
ENTRY_STYLE = {'background': BG_COLOR, 'foreground': FG_COLOR}
LABEL_STYLE = {'background': BG_COLOR, 'foreground': FG_COLOR}
MENU_STYLE = {'bg': BG_COLOR, 'fg': FG_COLOR, 'bd': 0,
			'activebackground': BG_COLOR, 'activeforeground': FG_COLOR}

BUTTON_STYLE = {'bg': BG_COLOR, 'activebackground': FG_COLOR, 'fg': FG_COLOR, 'activeforeground': BG_COLOR,
				'font': SMALL_FONT}
BIG_BUTTON_STYLE = {'height': 2, 'width': 10, 'borderwidth': 0, 'font': LARGE_FONT,
					'bg': BG_COLOR, 'fg': FG_COLOR, 'activebackground': FG_COLOR, 'activeforeground': BG_COLOR}

COLORED_FRAME = {'highlightbackground': FG_COLOR, 'highlightcolor': FG_COLOR,
				'highlightthickness': 2, 'bd': 0, 'bg': BG_COLOR}
