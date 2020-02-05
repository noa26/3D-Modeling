"""
helping functions that i don't know where to put.
"""


def beep(frequency=2500, duration=500):
    import winsound
    winsound.Beep(frequency, duration)
