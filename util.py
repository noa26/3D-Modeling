"""
helping functions that i don't know where to put.
"""


def beep(frequency=2500, duration=500):
    import winsound
    winsound.Beep(frequency, duration)


def serial_number(device):
    import pyrealsense2 as rs
    return int(device.get_info(rs.camera_info.serial_number))


beep(frequency=4000)
