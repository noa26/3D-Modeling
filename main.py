#####################################################
##                      MAIN                       ##
#####################################################

# Written by: Tal Adoni Nakar

# Explanation:
# - using threshold example to set a threshold on the depth frame
# - using another example (I don't remember which) to convert depth frame into point cloud and points
# - converting the point cloud into points and then into numpy array
# - using open3d to create a mesh from point cloud (using ball pivoting algorithm)
# - saving as a stl file (with open3d)

# imports
import pyrealsense2 as rs2
import open3d as o3d
import numpy as np
from math import sin, cos, radians

# The distance between the camera and the object
# (for setting the object's center as the origin center of the point cloud)
DISTANCE = 0.24
# debug boolean
DEBUG = False


def capture_point_cloud(number_of_frames: int = 1, camera_id: str = "", max_dist: float = 0.5, min_dist: float = 0.1)\
        -> np.numarray:
    """
    Start a pipeline with a depth stream configuration (and enable a given camera id, if given. else - takes depth
    frames from all connected cameras). Then, wait for frameset, apply a threshold filter, convert to depth frame,
    and pass it to a point cloud realsense2 object to process and get the points, which it returns as a numpy array.
    :param number_of_frames: The number of frames to sample.
    :param camera_id: The camera id to get the depth frame from.
    :param max_dist: the maximum distance between the camera and the object.
    :param min_dist: the minimum distance between the camera and the object.
    :return: point cloud's points as a numpy array.
    """
    # Initialize the numpy array that we will return
    vertices: np.numarray = np.asanyarray([])
    # Declare RealSense pipeline, encapsulating the actual device and sensor (depth sensor)
    pipe: rs2.pipeline = rs2.pipeline()
    config: rs2.config = rs2.config()
    # Enable depth stream
    config.enable_stream(rs2.stream.depth)
    # If there was given a camera_id, enable it in the configs
    if len(camera_id) == 12:
        config.enable_device(camera_id)
    # Start streaming with chosen configuration
    pipe.start(config)

    # Create a thresh filter with max distance of 0.7 meter
    thresh_filter: rs2.filter = rs2.threshold_filter(max_dist=max_dist, min_dist=min_dist)

    try:
        if DEBUG:
            print("Taking Picture")
        # Wait for 30 frames to make it more accurate
        for _ in range(30):
            pipe.wait_for_frames()

        # Wait for the next set of frames from the camera
        frames: rs2.composite_frame = pipe.wait_for_frames()

        if DEBUG:
            print("Processing Picture")

        # Apply the thresh filter on the frames we just got
        filtered_frames: rs2.composite_frame = thresh_filter.process(frames).as_frameset()

        # Convert the filtered frames to depth frame
        filtered_depth_frame: rs2.depth_frame = filtered_frames.get_depth_frame()

        # Declare realsense's pointcloud object, for calculating pointclouds
        rs_pc: rs2.pointcloud = rs2.pointcloud()

        # Get the point cloud's points of the filtered depth frame
        points: rs2.points = rs_pc.calculate(filtered_depth_frame)

        # Convert the points to a numpy array
        vertices: np.numarray = np.asanyarray(points.get_vertices())
        # Eliminate the origin points (there are a lot of (0,0,0) that just make the calculation time longer)
        vertices: np.numarray = np.asanyarray([[x, y, z] for (x, y, z) in vertices if not x == y == z == 0])
    finally:
        # stop the pipeline
        pipe.stop()
    # return the point cloud's points
    return vertices
    

def save_stl_from_pc(point_cloud: np.numarray, filename: str = "temp"):
    """
    Gets a numpy array of the point cloud, converting to open3d's point cloud, and using the ball pivoting algorithm
    to create a mesh out of the point cloud, and then saving the mesh into a stl file.
    :param point_cloud: a point cloud of the object, as a nu,py array of lists that contains x,y,z coordinates.
    :param filename: the filename to save the mesh into.
    :return: nothing, void.
    """
    # Make sure the filename ends with '.stl'
    if filename[-4:] != ".stl":
        filename += ".stl"

    if DEBUG:
        print("Getting Ready To Create A Mesh From Point Cloud")

    # Create a open3d's  point cloud object
    o3d_pc = o3d.geometry.PointCloud()
    # Convert the points numpy array to a open3d points
    o3d_pc.points = o3d.utility.Vector3dVector(point_cloud)
    # Estimate the point cloud's normals
    o3d_pc.estimate_normals()

    # Estimate radius for rolling ball
    distances = o3d_pc.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist

    if DEBUG:
        print("Creating Mesh From Point Cloud")

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        o3d_pc,
        o3d.utility.DoubleVector([radius, radius * 2]))

    if DEBUG:
        print("Saving Mesh")
    o3d.io.write_triangle_mesh(filename, mesh)
    if DEBUG:
        print("Saved")


def rotate_point_cloud(original_vertices: np.numarray, angle: int) -> np.numarray:
    """
    Rotate a given point cloud around the y axis with a given angle.
    The way to rotate a vertex is with matrix multiplication of (4x4) * (4x1) = (4x1):
    (cos(alpha)   0   -sin(alpha) 0) * (x) = (new x)
    (0            1   0           0) * (z) = (new z)
    (sin(alpha)   0   cos(alpha)  0) * (y) = (new y)
    (0            0   0           1) * (1) = (1    )
    :param original_vertices: the original point cloud, as a numpy array.
    :param angle: the angle, in degrees, to rotate the point cloud around the y axis, as an int.
    :return: the new rotated point cloud as a numpy array of vertices.
    """

    # initialize the rotated vertices list
    rotated_vertices_list = []

    # create the left matrix in the multiplication
    multi_matrix = np.asanyarray(
        [[cos(radians(angle)), 0, -sin(radians(angle)), 0],
         [0, 1, 0, 0],
         [sin(radians(angle)), 0, cos(radians(angle)), 0],
         [0, 0, 0, 1]])

    # for every vertex in the original vertex
    for (x, y, z) in original_vertices:
        # create the right matrix in the multiplication
        point_matrix = np.asanyarray([[x], [y], [z], [1]])

        # multiply the matrices (rotate the vertex)
        result_matrix = np.dot(multi_matrix, point_matrix)

        # get the nex vertex as a list of [x, y, z]
        new_vertex = [result_matrix[i][0] for i in range(3)]

        # add it to the rotated vertices list
        rotated_vertices_list.append(new_vertex)

    # return the rotated vertices as a numpy array
    return np.asanyarray(rotated_vertices_list)


def main():
    cameras = ['918512070565' for _ in range(6)]

    manipulated_vertices: np.numarray = np.asanyarray([[0, 0, 0]])

    for i in range(len(cameras)):
        camera_id = cameras[i]
        print("Capturing point cloud")
        # capture a point cloud
        original_vertices = capture_point_cloud(camera_id=camera_id, min_dist=0.1, max_dist=.2)
        # if
        # set the origin point to be the object's center (instead the camera's position)
        new_origin_vertices = np.asanyarray([[-x, y, DISTANCE-z] for (x, y, z) in original_vertices])

        print("Manipulating the point cloud")

        manipulated_vertices = np.append(manipulated_vertices, rotate_point_cloud(new_origin_vertices, 60 * i), axis=0)

        # print("Save 1")
        # save_stl_from_pc(original_vertices, "original_origin_point.stl")
        # print("Save 2")
        # save_stl_from_pc(new_origin_vertices, "new_origin_point.stl")
        # print("Save 3")
    save_stl_from_pc(manipulated_vertices, f"final.stl")

    print("Finished")


if __name__ == "__main__":
    main()
