"""

"""

import pyrealsense2 as rs2
import numpy as np
import cv2


def capture(pipe: rs2.pipeline) -> None:
    """

    :param pipe: a running pipeline
    :return: None
    """

    window_name = 'RealSense'
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)  # Init a window
    colorizer: rs2.colorizer = rs2.colorizer()

    while cv2.waitKey(1) < 0:

        # Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        # depth_image = np.asanyarray(depth_frame.get_data())  # Not necessary with new conversion
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.imshow(window_name, images)


if __name__ == "__main__":
    pipe = rs2.pipeline()
    config = rs2.config()
    config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)  # Format the depth channel from the camera
    config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)  # Format the color channel from the camera
    try:
        pipe.start(config)
        capture(pipe)
    finally:
        pipe.stop()
