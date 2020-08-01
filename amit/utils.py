from math import cos, radians, sin
from typing import List, Any, Callable, Tuple

import numpy as np
import open3d as o3d
import pyrealsense2 as rs2


def get_filter(filter_name: str, *args) -> rs2.filter_interface:
    """
    Basically a factory for filters (Maybe change to Dict 'cause OOP)
    :param filter_name: The filter name
    :param args: The arguments for the filter
    :return: A filter which corresponds to the filter name with it's arguments
    """
    if filter_name == 'decimation_filter':
        return rs2.decimation_filter(*args)
    elif filter_name == 'threshold_filter':
        return rs2.threshold_filter(*args)
    elif filter_name == 'disparity_transform':
        return rs2.disparity_transform(*args)
    elif filter_name == 'spatial_filter':
        return rs2.spatial_filter(*args)
    elif filter_name == 'temporal_filter':
        return rs2.temporal_filter(*args)
    elif filter_name == 'hole_filling_filter':
        return rs2.hole_filling_filter(*args)
    else:
        raise Exception(f'The filter \'{filter_name}\' does not exist!')


def get_filters(filters_name_args: List[List[Any]]) -> List[rs2.filter_interface]:
    return [get_filter(filter_name, *args) for (filter_name, *args) in filters_name_args]


def change_coordinates_inplace(points: np.ndarray,
                               f: Callable[[Tuple[float, float, float]], Tuple[float, float, float]]) -> None:
    """
    Change the coordinates of every point inplace via a function that returns the new coordinate of each point.
    :param points: The point cloud to change the coordinates of
    :param f: The function that the coordinates will change by
    :return: None
    """

    assert len(points.shape) == 2
    assert points.shape[1] == 3

    for i, v in enumerate(points):
        res_point = f(v)
        for j in range(points.shape[1]):
            points[i][j] = res_point[j]


def filter_points(points: np.ndarray, pred: Callable[[Tuple[float, float, float]], bool]):
    """

    :param points: The point cloud to filter points from
    :param pred: A predicate to indicate whether to remove a point or not (if True, it is removed)
    :return: A new point cloud without the filtered points
    """
    assert len(points.shape) == 2
    assert points.shape[1] == 3

    del_idx = []

    for i, p in enumerate(points):
        if pred(p):  # predicate that return true if the points are removed
            del_idx.append(i)

    return np.delete(points, del_idx, axis=0)


def rotate_point_cloud_inplace(points: np.ndarray, angle: float) -> None:
    """
    Rotate a given point cloud around the y axis with a given angle inplace.
    The way to rotate a vertex is with matrix multiplication of (4x4) * (4x1) = (4x1):
    (cos(alpha)   0   -sin(alpha) 0) * (x) = (new x)
    (0            1   0           0) * (y) = (new y)
    (sin(alpha)   0   cos(alpha)  0) * (z) = (new z)
    (0            0   0           1) * (1) = (1    )
    :param points: The point cloud to rotate
    :param angle: The angle, in degrees, to rotate the point cloud around the y axis
    """

    assert len(points.shape) == 2
    assert points.shape[1] == 3

    # Create the left matrix in the multiplication
    rotate_matrix = np.array(
        [[cos(radians(angle)), 0, -sin(radians(angle)), 0],
         [0, 1, 0, 0],
         [sin(radians(angle)), 0, cos(radians(angle)), 0],
         [0, 0, 0, 1]])

    for i, (x, y, z) in enumerate(points):
        # Create an extended vector to perform the matrix multiplication
        ext_vec = np.array([[x], [y], [z], [1]])

        # Multiply the matrices (rotate the vertex)
        res_vec = np.dot(rotate_matrix, ext_vec)

        # Assign the result
        for j in range(points.shape[1]):
            points[i][j] = res_vec[j]


def save_file(points: np.ndarray, filename: str) -> None:
    """
    Save a point cloud to a file
    :param points: The point cloud to save to a file
    :param filename:The filename
    :return: None
    """

    o3d_pc = o3d.geometry.PointCloud()

    o3d_pc.points = o3d.utility.Vector3dVector(points)
    o3d_pc.estimate_normals()

    distances = o3d_pc.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        o3d_pc,
        o3d.utility.DoubleVector([radius, radius * 2]))

    o3d.io.write_triangle_mesh(filename, mesh)


def get_intrinsics(config: rs2.config) -> rs2.intrinsics:
    """

    :param config: The config to get the intrinsics
    :return: The intrinsics of the camera
    """
    pipe = rs2.pipeline()
    try:
        profile = pipe.start(config)
        return profile.get_stream(rs2.stream.depth).as_video_stream_profile().get_intrinsics()
    finally:
        pipe.stop()

