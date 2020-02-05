"""

"""

import pyrealsense2 as rs
import numpy as np
import cv2


def main():
    pipe = None
    try:
        pipe = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        pipe.start(config)

        capture(pipe)

    finally:
        pipe.stop()


def capture(pipe: rs.pipeline):
    """

    :param pipe: a running pipeline
    :return: a numpy array of one depth frame.
    """
    print(pipe.get_active_profile())

    frames = pipe.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        print("---problem---")
        return None

    # Convert image to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    images = np.hstack((color_image, depth_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    x = input("stop?")
    print(x)


if __name__ == "__main__":
    main()
