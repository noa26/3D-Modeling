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

# https://stackoverflow.com/questions/56965268/how-do-i-convert-a-3d-point-cloud-ply-into-a-mesh-with-faces-and-vertices

# imports
import pyrealsense2 as rs2
import open3d as o3d
import numpy as np
from math import sin, cos, radians
from typing import List, Tuple
from pointcloud_creation import average_matrices_to_pointcloud, depthframe_to_depthmatrix

# The distance between the camera and the object
# (for setting the object's center as the origin center of the point cloud)
# ((.695+.69+.69+.68+.68+.67)/6)-3.5 = .6492
DISTANCE = 0.6492
# debug boolean
DEBUG = True
# stl file name
FILENAME = "FINALE.stl"


def capture_point_cloud(number_of_frames: int = 1, camera_id: str = "", max_dist: float = 0.5, min_dist: float = 0.1) \
		-> Tuple[List[np.ndarray], rs2.intrinsics]:
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
	vertices_list: np.numarray = np.asanyarray([])
	# Declare RealSense pipeline, encapsulating the actual device and sensor (depth sensor)
	pipe: rs2.pipeline = rs2.pipeline()
	config: rs2.config = rs2.config()
	# Enable depth stream
	config.enable_stream(rs2.stream.depth)
	# If there was given a camera_id, enable it in the configs
	if len(camera_id) == 12:
		config.enable_device(camera_id)
	# Start streaming with chosen configuration
	pipeline_profile = pipe.start(config)

	# Create a thresh filter with max distance of 0.7 meter
	thresh_filter: rs2.filter = rs2.threshold_filter(max_dist=max_dist, min_dist=min_dist)

	try:
		if DEBUG:
			print("Taking Picture")
		# Wait for 30 frames to make it more accurate
		for _ in range(30):
			pipe.wait_for_frames()

		# Wait for the next set of frames from the camera
		frames: List[rs2.composite_frame] = []
		for _ in range(number_of_frames):
			frames.append(pipe.wait_for_frames().as_frameset().get_depth_frame().as_depth_frame())

		if DEBUG:
			print("Processing Picture")

		return ([depthframe_to_depthmatrix(frame) for frame in frames], pipeline_profile.get_stream(
			rs2.stream.depth).as_video_stream_profile().get_intrinsics())

		# For every captured frame
		# for frame in frames:
		#     # Apply the thresh filter on the frames we just got
		#     filtered_frames: rs2.composite_frame = thresh_filter.process(frame).as_frameset()
		#
		#     # Convert the filtered frames to depth frame
		#     filtered_depth_frame: rs2.depth_frame = filtered_frames.get_depth_frame()
		#
		#     # Declare realsense's pointcloud object, for calculating pointclouds
		#     rs_pc: rs2.pointcloud = rs2.pointcloud()
		#
		#     # Get the point cloud's points of the filtered depth frame
		#     points: rs2.points = rs_pc.calculate(filtered_depth_frame)
		#
		#     # Convert the points to a numpy array
		#     vertices: np.numarray = np.asanyarray(points.get_vertices())
		#     # Eliminate the origin points (there are a lot of (0,0,0) that just make the calculation time longer)
		#     vertices: np.numarray = np.asanyarray([[x, y, z] for (x, y, z) in vertices if not x == y == z == 0])
		#
		#     vertices_list = vertices_list.append(vertices)
	finally:
		# stop the pipeline
		pipe.stop()
	# # return the point cloud's points
	# return vertices_list


def capture_point_cloud2(number_of_frames: int = 1, camera_id: str = "", max_dist: float = 0.5, min_dist: float = 0.1) \
		-> Tuple[List[rs2.depth_frame], rs2.intrinsics]:
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
	# vertices_list: np.numarray = np.asanyarray([])
	# Declare RealSense pipeline, encapsulating the actual device and sensor (depth sensor)
	pipe: rs2.pipeline = rs2.pipeline()
	config: rs2.config = rs2.config()
	# Enable depth stream
	config.enable_stream(rs2.stream.depth)
	# If there was given a camera_id, enable it in the configs
	if len(camera_id) == 12:
		config.enable_device(camera_id)
	# Start streaming with chosen configuration
	pipeline_profile = pipe.start(config)

	# Create a thresh filter with max distance of 0.7 meter
	thresh_filter: rs2.threshold_filter = rs2.threshold_filter(max_dist=max_dist, min_dist=min_dist)
	spatial_filter: rs2.spatial_filter = rs2.spatial_filter()

	try:
		if DEBUG:
			print("Taking Picture")
		# Wait for 30 frames to make it more accurate
		for _ in range(30):
			pipe.wait_for_frames()

		# Wait for the next set of frames from the camera
		frames: List[rs2.composite_frame] = []
		for _ in range(number_of_frames):
			frames.append(pipe.wait_for_frames().as_frameset().get_depth_frame().as_depth_frame())

		if DEBUG:
			print("Processing Picture")

		return ([depthframe_to_depthmatrix(frame) for frame in frames], pipeline_profile.get_stream(
			rs2.stream.depth).as_video_stream_profile().get_intrinsics())

		# For every captured frame
		# for frame in frames:
		#     # Apply the thresh filter on the frames we just got
		#     filtered_frames: rs2.composite_frame = thresh_filter.process(frame).as_frameset()
		#
		#     # Convert the filtered frames to depth frame
		#     filtered_depth_frame: rs2.depth_frame = filtered_frames.get_depth_frame()
		#
		#     # Declare realsense's pointcloud object, for calculating pointclouds
		#     rs_pc: rs2.pointcloud = rs2.pointcloud()
		#
		#     # Get the point cloud's points of the filtered depth frame
		#     points: rs2.points = rs_pc.calculate(filtered_depth_frame)
		#
		#     # Convert the points to a numpy array
		#     vertices: np.numarray = np.asanyarray(points.get_vertices())
		#     # Eliminate the origin points (there are a lot of (0,0,0) that just make the calculation time longer)
		#     vertices: np.numarray = np.asanyarray([[x, y, z] for (x, y, z) in vertices if not x == y == z == 0])
		#
		#     vertices_list = vertices_list.append(vertices)
	finally:
		# stop the pipeline
		pipe.stop()
	# # return the point cloud's points
	# return vertices_list


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

	print(point_cloud)

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


def capture_object_to_stl_from_multiple_cameras(cameras: List[Tuple[str, int, float]]):
	all_vertices: np.numarray = np.asanyarray([[0, 0, 0]])

	for camera_id, angle, dist in cameras:
		if DEBUG:
			print(camera_id)

		# capture a point cloud
		frames_list, intern = capture_point_cloud(number_of_frames=15, camera_id=camera_id, min_dist=0.1,
		                                          max_dist=dist + 0.1)

		# get average point cloud from collected frames
		original_vertices = average_matrices_to_pointcloud(frames_list, intern, min_dist=0.1, max_dist=dist + 0.1)

		# set the origin point to be the object's center (instead the camera's position)
		# fixed_vertices = np.asanyarray([[x-0.015, y, z] for (x, y, z) in original_vertices])

		# set the origin point to be the object's center (instead the camera's position)
		new_origin_vertices = np.asanyarray([[-x, y, dist - z] for (x, y, z) in original_vertices])

		if DEBUG:
			print("Manipulating the point cloud")

		manipulated_vertices = rotate_point_cloud(new_origin_vertices, angle)

		save_stl_from_pc(manipulated_vertices, f"{camera_id}.stl")

		if DEBUG:
			print("Adding to all_vertices")

		all_vertices = np.append(all_vertices, manipulated_vertices, axis=0)

	save_stl_from_pc(all_vertices, "END-TEST-NUM-1.stl")


def main():
	# import util
	# cameras = [('918512071186', 0, .66), ('918512070565', 60, .625), ('918512072325', 120, .625),
	#            ('918512073384', 180, .655), ('815412071199', 240, .66), ('815412070919', 300, .67)]

	# cameras = [('918512071186', 0, .9), ('918512073384', 60, .9), ('918512070565', 120, .9),
	#            ('815412070919', 180, .9), ('815412071199', 240, .9)]

	# 87
	#
	# master - 918512071186
	# 2 - 918512072325
	# 3 - 918512070565
	# 4 - 815412070919
	# 5 - 815412071199
	# 6 - 918512073384

	cameras = [('918512071186', 0, 0.87), (918512072325, 60, 0.87), (918512070565, 120, 0.87),
	           ('815412070919', 180, 0.87), (815412071199, 240, 0.87), (918512073384, 300, 0.87)]

	# cameras = [('918512071186', 90*i, 0.87) for i in range(4)]

	cameras = [(str(cam_id), angle, dist) for (cam_id, angle, dist) in cameras]

	# get first frames from each camera
	# for camera in cameras:
	#     util.get_dummy_frames(int(camera[0]))

	# cameras = [('918512071186', 0, .7), ('918512070565', 60, .7), ('918512072325', 120, .7),
	#            ('918512073384', 180, .7), ('815412071199', 240, .7), ('815412070919', 300, .7)]

	# cameras = [('918512071186', 0, .66), ('918512073384', 180, .66)]

	# cameras = [('918512070565', 60, .625), ('815412071199', 240, .66)]

	capture_object_to_stl_from_multiple_cameras(cameras)


if __name__ == "__main__":
	main()
