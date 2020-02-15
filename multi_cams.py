"""
this code was written by marialarsson and published on:
https://github.com/IntelRealSense/librealsense/issues/1735
"""
from collections import Iterable

import pyrealsense2 as rs2
import numpy as np
import cv2

import util


def show_multi_cam(pipes: Iterable) -> None:
    colorizer: rs2.colorizer = rs2.colorizer()

    while cv2.waitKey(1) < 0:
        for pipe in pipes:
            # Camera 1
            # Wait for a coherent pair of frames: depth and color
            frames = pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            images = np.hstack((color_image, depth_colormap))
            win_name = 'RealSense {}'.format(str(pipe))
            cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win_name, images)


if __name__ == '__main__':
    pipeline_1 = rs2.pipeline()
    config_1 = rs2.config()
    config_1.enable_device('918512071186')
    config_1.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
    config_1.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
    # ...from Camera 2
    pipeline_2 = rs2.pipeline()
    config_2 = rs2.config()
    config_2.enable_device('918512072325')
    config_2.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
    config_2.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)

    try:
        # Start streaming from both cameras
        print(util.serial_number(pipeline_1.start(config_1).get_device()))
        print(util.serial_number(pipeline_2.start(config_2).get_device()))

        show_multi_cam([pipeline_1, pipeline_2])
    finally:
        pipeline_1.stop()
        pipeline_2.stop()
