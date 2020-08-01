WHITE = '#dbdadf'
GREY = '#201f24'
BLUE = '#87ceec'
RED = '#FF6347'

BG_COLOR = GREY
FG_COLOR = BLUE
TEXT_COLOR = WHITE

LARGE_FONT = ('Bahnschrift light', 25)
SMALL_FONT = ('Bahnschrift light', 10)

WINDOW_STYLE = {'background': BG_COLOR}
FRAME_STYLE = {'background': BG_COLOR}
CANVAS_STYLE = {'background': BG_COLOR}
LABEL1_STYLE = {'font': SMALL_FONT, 'background': BG_COLOR, 'foreground': TEXT_COLOR}
LABEL2_STYLE = {'font': SMALL_FONT, 'background': BG_COLOR, 'foreground': FG_COLOR}
ENTRY_STYLE = {'font': SMALL_FONT, 'background': BG_COLOR, 'foreground': TEXT_COLOR, 'insertbackground': FG_COLOR}

MENU_STYLE = {'font': SMALL_FONT, 'background': BG_COLOR, 'foreground': TEXT_COLOR,
              'bd': 0, 'activebackground': BG_COLOR, 'activeforeground': FG_COLOR}

CHECKBUTTON_STYLE = {'font': SMALL_FONT, 'background': BG_COLOR, 'foreground': TEXT_COLOR,
                     'activebackground': FG_COLOR, 'activeforeground': BG_COLOR,
                     'selectcolor': BG_COLOR}

BUTTON_STYLE = {'font': SMALL_FONT, 'background': BG_COLOR, 'foreground': FG_COLOR,
                'activebackground': FG_COLOR, 'activeforeground': BG_COLOR}

BIG_BUTTON_STYLE = {'font': LARGE_FONT, 'background': BG_COLOR, 'foreground': FG_COLOR,
                    'activebackground': FG_COLOR, 'activeforeground': BG_COLOR,
                    'height': 2, 'width': 10, 'borderwidth': 0}

COLORED_FRAME = {'background': BG_COLOR, 'highlightbackground': FG_COLOR,
                 'highlightcolor': FG_COLOR, 'highlightthickness': 2}
