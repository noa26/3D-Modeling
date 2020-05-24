from typing import List

import numpy as np
import pyrealsense2 as rs2


def depthmatrix_to_pointcloud(matrix: np.ndarray, intrin: rs2.intrinsics, max_dist=float('Inf'),
                              min_dist=float('-Inf')) -> np.ndarray:
    """
    Uses rs2_deproject_pixel_to_point to map each pixel to a point.
    Note: i, j are flipped 'cause realsense is stupid
    :param matrix: The matrix that represents the depth frame
    :param intrin: The intrinsics of the camera
    :param max_dist: the maximum distance
    :param min_dist: the maximum distance
    :return: A pointcloud via the depth frame
    """
    assert len(matrix.shape) == 2

    points: List = []
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            if min_dist <= matrix[i][j] <= max_dist:
                points.append(rs2.rs2_deproject_pixel_to_point(intrin, [j, i], matrix[i][j]))

    return np.array(points)


def depthframe_to_depthmatrix(frame: rs2.depth_frame) -> np.ndarray:
    """
    Data in depthframe is the distance in millimeter (rounded to the millimeter level).
    Note: We can get better precision using the depth.get_distance method,
        so for easier upgradeability this is a function.
    :param frame: the depthframe to convert to matrix
    :return: a matrix that represents the frame
    """
    return np.array(frame.get_data()) / 1000  # divide by 1000 to get meters


if __name__ == '__main__':
    """
    Here I tried the average 10 frames but for some reason it got a shit ton of noise LOL
    """

    pipe: rs2.pipeline = rs2.pipeline(rs2.context())

    cfg = rs2.config()
    cfg.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)  # Format the depth channel from the camera
    cfg.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)  # Format the color channel from the camera

    try:
        pipeline_profile: rs2.pipeline_profile = pipe.start(cfg)

        intrin: rs2.intrinsics = pipeline_profile.get_stream(
            rs2.stream.depth).as_video_stream_profile().get_intrinsics()

        # Reading some frames before the real capture should make the quality better
        for _ in range(30):
            pipe.wait_for_frames()

        all_frames = []
        num_frames = 10

        # Saves 'num_frames' depthframe to matrix
        for i in range(num_frames):
            frames: rs2.composite_frame = pipe.wait_for_frames()
            depth_frame: rs2.depth_frame = frames.as_frameset().get_depth_frame().as_depth_frame()
            all_frames.append(depthframe_to_depthmatrix(depth_frame))

        # Write each frame to file for testing
        for i, frame in enumerate(all_frames):
            pc: np.ndarray = depthmatrix_to_pointcloud(frame, intrin, max_dist=0.7, min_dist=0.1)
            with open(f'pc_test_{i}.xyz', 'w') as f:
                for point in pc:
                    f.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n')

        # Write average frame to file for testing
        pc = depthmatrix_to_pointcloud(sum(all_frames) / num_frames, intrin, max_dist=0.7, min_dist=0.1)
        with open('pc_test_avg.xyz', 'w') as f:
            for point in pc:
                f.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n')

    finally:
        pipe.stop()
