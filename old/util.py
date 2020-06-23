"""
helping functions that i don't know where to put.
"""
import pyrealsense2 as rs2


def beep(frequency=2500, duration=500) -> None:
	import winsound
	winsound.Beep(frequency, duration)


def serial_number(device: rs2.device) -> str:
	return device.get_info(rs2.camera_info.serial_number)


def get_dummy_frames(sn: int, count: int = 30):
	config: rs2.config = rs2.config()
	config.enable_device(str(sn))
	config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
	config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
	pipe: rs2.pipeline = rs2.pipeline(config)
	try:
		pipe.start()
		for _ in range(count):
			pipe.wait_for_frames()
	finally:
		pipe.stop()
