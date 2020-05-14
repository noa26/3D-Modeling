#####################################################
##                  Export to STL                  ##
#####################################################

# Written by: Tal Adoni Nakar


# Explanation:
# - using threshold example to set a threshold on the depth frame
# - using export_ply_example.py to create a ply file
#   https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/export_ply_example.py
# - using openmash to convert from ply to stl
#   https://github.com/intel-isl/Open3D/issues/867


# imports
import pyrealsense2 as rs2
import openmesh as om
from os import remove

# get the file name of the stl file (and make sure it ends with '.stl')
filename = input("Enter the file name of the stl file: ")
if filename[-4:] != ".stl":
    filename += ".stl"

print("Taking a depth frame")

# declare pointcloud object, for calculating pointclouds and texture mappings
pc: rs2.pointcloud = rs2.pointcloud()
# we want the points object to be persistent so we can display the last cloud when a frame drops
points: rs2.points = rs2.points()

# declare RealSense pipeline, encapsulating the actual device and sensors
pipe: rs2.pipeline = rs2.pipeline()
config: rs2.config = rs2.config()
# enable depth stream
config.enable_stream(rs2.stream.depth)

# start streaming with chosen configuration
pipe.start(config)

# we'll use the colorizer to generate texture for our PLY
# (alternatively, texture can be obtained from color or infrared stream)
colorizer: rs2.colorizer = rs2.colorizer()

# create a thresh filter with max distance of 0.7 meter
thresh_filter: rs2.filter = rs2.threshold_filter(max_dist=0.9, min_dist=0.1)


try:
    # wait for the next set of frames from the camera
    frames: rs2.composite_frame = pipe.wait_for_frames()

    print("Applying thresh filter")
    # apply the thresh filter on the frame we just got
    filtered_frames: rs2.depth_frame = thresh_filter.process(frames)

    # colorize the filtered frames
    colorized = colorizer.process(filtered_frames)

    print("Creating ply file")
    # create save_to_ply object
    ply: rs2.save_to_ply = rs2.save_to_ply("temp.ply")

    # set options to the desired values
    # w'll generate a textual PLY with binary (mesh is already created by default)
    ply.set_option(rs2.save_to_ply.option_ply_binary, True)
    ply.set_option(rs2.save_to_ply.option_ply_normals, False)

    print("Saving as ply file")
    # apply the processing block to the frameset which contains the depth frame and the texture
    ply.process(colorized)
    print("Finished saving as ply file")

    # -- convert from ply to stl
    print("Converting to stl file")
    # read the mesh from the ply file
    mesh = om.read_trimesh("temp.ply")
    # save it as a stl file
    om.write_mesh(filename, mesh)
    print("Finished converting to stl file")

    # remove the ply file
    remove("temp.ply")
    print("Removed the ply file")

finally:
    pipe.stop()

print("Finished")
