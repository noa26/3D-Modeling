import cv2
import json
import numpy as np
import pyrealsense2 as rs2
from typing import List


with open('config.json') as f:
    cfg_map = json.load(f)
cams = cfg_map['cameras']


def capture(entries, listener):
    print('filetype:', entries['file_type'].get())
    print('filename:', entries['file_name'].get())
    print("unimplemented\tcapture")
    pass


def available_cameras() -> List[str]:
    return ['a' * 20, 'b' * 20]
    # def serial_number(device: rs2.device) -> str:
    #     return device.get_info(rs2.camera_info.serial_number)
    #
    # ctx = rs2.context()
    # return [serial_number(device) for device in ctx.devices]


def camera_display(serial_num: str) -> int:
    pipe = rs2.pipeline()
    config_1 = rs2.config()
    config_1.enable_device(serial_num)
    config_1.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
    colorizer: rs2.colorizer = rs2.colorizer()

    try:
        pipe.start()
        while cv2.waitKey(1) < 0:
            # Wait for coherent depth frames
            frames = pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            win_name = 'RealSense ' + serial_num
            cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win_name, depth_colormap)
        cv2.destroyAllWindows()
        return 0
    except:
        return -1
