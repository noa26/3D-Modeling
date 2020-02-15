"""
helping functions that i don't know where to put.
"""
import pyrealsense2 as rs2


def beep(frequency=2500, duration=500) -> None:
    import winsound
    winsound.Beep(frequency, duration)


def serial_number(device: rs2.device) -> str:
    return device.get_info(rs2.camera_info.serial_number)
