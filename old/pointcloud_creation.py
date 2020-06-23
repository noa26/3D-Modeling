from typing import List, Dict, Tuple

import numpy as np
import pyrealsense2 as rs2
from collections import defaultdict

from old.util import beep


def average_matrices_to_pointcloud(mats: List[np.ndarray], intrin: rs2.intrinsics, max_dist=float('Inf'),
                                   min_dist=float('-Inf')) -> np.ndarray:
    """
    Uses rs2_deproject_pixel_to_point as in the depthmatrix_to_pointcloud function.
    Points the have the same i,j values have their z values averaged out, iff the z value is within the range.
    :param mats: A list of matrices that each represents the depth frame
    :param intrin: The intrinsics of the camera
    :param max_dist: the maximum distance
    :param min_dist: the maximum distance
    :return: A pointcloud calculated via averaging the matrices and the rs2_deproject_pixel_to_point function.
    """
    ij_to_zs: Dict[Tuple[int, int], List[float]] = defaultdict(list)
    for mat in mats:
        assert len(mat.shape) == 2

        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                if min_dist <= mat[i][j] <= max_dist:
                    ij_to_zs[(i, j)].append(mat[i][j])

    pc = []
    for (i, j), zs in ij_to_zs.items():
        pc.append(rs2.rs2_deproject_pixel_to_point(intrin, [j, i], np.average(zs)))

    return np.array(pc)


def average_pointclouds(pcs: List[np.ndarray]) -> np.ndarray:
    """

    :param pcs: A list of pointclouds
    :return: A new point cloud where points with the x,y values have their z values averaged out
    """
    xy_to_zs: Dict[Tuple[float, float], List[float]] = defaultdict(list)
    for pc in pcs:
        for point in pc:
            xy_to_zs[(point[0], point[1])].append(point[2])

    new_pc = []
    for (x, y), zs in xy_to_zs.items():
        new_pc.append([x, y, np.average(zs)])

    return np.array(new_pc)


def depthmatrix_to_pointcloud(matrix: np.ndarray, intrin: rs2.intrinsics, max_dist=float('Inf'),
                              min_dist=float('-Inf')) -> np.ndarray:
    """
    Uses rs2_deproject_pixel_to_point to map each pixel to a point.
    Note: i, j are flipped 'cause realsense is stupid
    :param matrix: The matrix that represents the depth frame
    :param intrin: The intrinsics of the camera
    :param max_dist: the maximum distance
    :param min_dist: the maximum distance
    :return: A pointcloud via the depth frame and the rs2_deproject_pixel_to_point function
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

        all_mats = []
        num_frames = 10

        # Saves 'num_frames' depthframe to matrix
        for i in range(num_frames):
            frames: rs2.composite_frame = pipe.wait_for_frames()
            depth_frame: rs2.depth_frame = frames.as_frameset().get_depth_frame().as_depth_frame()
            mat = depthframe_to_depthmatrix(depth_frame)
            all_mats.append(mat)

        beep()

        all_pcs = [depthmatrix_to_pointcloud(mat, intrin, max_dist=0.7, min_dist=0.1) for mat in all_mats]

        # Write each frame to file for testing
        for i, pc in enumerate(all_pcs):
            with open(f'pc_test_{i}.xyz', 'w') as f:
                for point in pc:
                    f.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n')

        # Averaging after calculating pcs (with thresholds),Works;
        # (Allot of points though, and may 'case weird behavior).
        avg_pc_1 = average_pointclouds(all_pcs)
        with open('pc_test_avg_1.xyz', 'w') as f:
            for point in avg_pc_1:
                f.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n')

        # Averaging mats and then calculating pc, Doesn't Work.
        avg_pc_2 = depthmatrix_to_pointcloud(np.average(all_mats, axis=0), intrin, max_dist=0.7, min_dist=0.1)
        with open('pc_test_avg_2.xyz', 'w') as f:
            for point in avg_pc_2:
                f.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n')

        # Averaging mats while [?] calculating pc, Works.
        avg_pc_3 = average_matrices_to_pointcloud(all_mats, intrin, max_dist=0.7, min_dist=0.1)
        with open('pc_test_avg_3.xyz', 'w') as f:
            for point in avg_pc_3:
                f.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n')

    finally:
        pipe.stop()
