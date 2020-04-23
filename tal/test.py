from collections import Iterable#, List

import pyrealsense2 as rs2
import numpy as np
import cv2

import util


def show_multi_cam(pipes: Iterable) -> None:
    colorizer: rs2.colorizer = rs2.colorizer()

    while cv2.waitKey(1) < 0:
        for pipe, _ in pipes:
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
            images = np.vstack((depth_colormap, color_image))
            win_name = 'RealSense {}'.format(str(pipe))
            cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win_name, images)


def capture_one_cam(cam_id: str):
    pipe = rs2.pipeline()
    config = rs2.config()
    config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)  # Format the depth channel from the camera
    config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)  # Format the color channel from the camera

    try:
        pipe.start(config)
        # Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        while not depth_frame and not color_frame:
            # Wait for a coherent pair of frames: depth and color
            frames = pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and
        # approximating the coverage of pixels within one meter
        coverage = [0] * 64
        for y in range(480):
            for x in range(640):
                dist = depth_frame.get_distance(x, y)
                if 0 < dist < 1:
                    coverage[int(x / 10)] += 1

            if y % 20 is 19:
                line = ""
                for c in coverage:
                    line += " .:nhBXWZ"[int(c / 25)]
                coverage = [0] * 64
                print(line)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)
        colorizer: rs2.colorizer = rs2.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

    finally:
        pipe.stop()


def get_cams_pipes(cams_id: Iterable): # -> List[rs2.pipeline]:
    pipes = []
    for cam_id in cams_id:
        pipeline = rs2.pipeline()
        config = rs2.config()
        config.enable_device(cam_id)
        config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
        config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
        pipes.append((pipeline, config))
    return pipes


def get_cams_info(cams_file: str): # -> List[str]:
    cams_info = []
    with open(cams_file, 'r') as f:
        for line in f:
            cams_info.append(line)
    return cams_info


if __name__ == '__main__':
    cams_ids = get_cams_info("cameras")
    # pipelines = get_cams_pipes(cams_ids)
    #
    # try:
    #     # Start streaming from both cameras
    #     for pipeline, config in pipelines:
    #         print(util.serial_number(pipeline.start(config).get_device()))
    #
    #     show_multi_cam(pipelines)
    #
    # finally:
    #     # Start streaming from both cameras
    #     for pipeline, _ in pipelines:
    #         pipeline.stop()

    # capture_one_cam(cams_ids[0])
    pipe = rs2.pipeline()
    pipe.start()
