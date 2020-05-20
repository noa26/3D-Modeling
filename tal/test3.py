import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
# from pyntcloud.pyntcloud import PyntCloud # open source library for 3D pointcloud visualisation
import pyrealsense2 as rs2                 # Intel RealSense cross-platform open-source API
print("Environment Ready")


# librealsense/wrappers/python/examples/export_ply_example.py
# https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/export_ply_example.py
def export_to_ply():
    # Declare pointcloud object, for calculating pointclouds and texture mappings
    pc = rs2.pointcloud()
    # We want the points object to be persistent so we can display the last cloud when a frame drops
    points = rs2.points()

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs2.pipeline()
    config = rs2.config()
    # Enable depth stream
    # config.enable_stream(rs2.stream.depth)
    config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
    config.enable_stream(rs2.stream.color, 640, 480, rs2.format.rgb8, 30)

    # Start streaming with chosen configuration
    pipe.start(config)

    # We'll use the colorizer to generate texture for our PLY
    # (alternatively, texture can be obtained from color or infrared stream)
    colorizer = rs2.colorizer()
    colorizer.set_option(rs2.option.color_scheme, 0)

    try:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        colorized = colorizer.process(frames)

        # Create save_to_ply object
        ply = rs2.save_to_ply("1.ply")

        # Set options to the desired values
        # In this example we'll generate a textual PLY with normals (mesh is already created by default)
        ply.set_option(rs2.save_to_ply.option_ply_binary, True)
        ply.set_option(rs2.save_to_ply.option_ply_normals, False)

        print("Saving to 1.ply...")
        # Apply the processing block to the frameset which contains the depth frame and the texture
        ply.process(colorized)
        print("Done")
    finally:
        pipe.stop()


# https://forums.intel.com/s/question/0D50P0000490UT0SAM/how-to-generate-pointclouds-and-draw-it-in-python?language=en_US
def intel_community():

    # create pipeline
    pipeline = rs2.pipeline()
    # create config
    config = rs2.config()
    # enable color and depth streams
    config.enable_stream(rs2.stream.color, 640, 480, rs2.format.rgb8, 30)
    config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
    # start the pipeline with the config
    pipeline.start(config)

    while True:
        # get frames from the pipeline
        frames = pipeline.wait_for_frames()
        # get color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if (not depth_frame) or (not color_frame):
            print("Color or Depth not available!")
            break

        # color
        color_img = np.asanyarray(color_frame.get_data())
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

        # create colorizer
        colorizer = rs2.colorizer()
        # colorize the depth frame
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # create a point cloud
        pc = rs2.pointcloud()
        # calculate point cloud out of depth frame
        points = pc.calculate(depth_frame)
        # map the point cloud to the color frame
        pc.map_to(color_frame)


def git_shit():
    pipeline = rs2.pipeline()
    config = rs2.config()

    config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 15)
    config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 15)
    config.enable_stream(rs2.stream.infrared, 640, 480, rs2.format.y8, 15)

    cfg = pipeline.start(config)
    dev = cfg.get_device()

    colorizer = rs2.colorizer()
    colorizer.set_option(rs2.option.color_scheme, 0)

    depth_sensor = dev.first_depth_sensor()
    depth_sensor.set_option(rs2.option.visual_preset, 2)

    try:
        while True:

            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            ir1_frame = frames.get_infrared_frame(1)
            if not depth_frame or not color_frame or not ir1_frame:
                continue

            depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir1_image = np.asanyarray(ir1_frame.get_data())

            cv2.namedWindow('RGB_RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Depth_RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('IR_RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RGB_RealSense', color_image)
            cv2.imshow('Depth_Realsense', depth_image)
            cv2.imshow('IR_RealSense', ir1_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        pipeline.stop()


pipe = rs2.pipeline()
cfg = rs2.config()
# cfg.enable_device_from_file("object_detection.bag")
profile = pipe.start(cfg)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(5):
    pipe.wait_for_frames()

# Store next frameset for later processing:
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()

# Cleanup:
pipe.stop()
print("Frames Captured")
# ---------------------------------------------------------------------
color = np.asanyarray(color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.imshow(color)
# ---------------------------------------------------------------------
colorizer = rs2.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
plt.imshow(colorized_depth)
# ---------------------------------------------------------------------
pc = rs2.pointcloud()
pc.map_to(color_frame)
pointcloud = pc.calculate(depth_frame)
pointcloud.export_to_ply("1.ply", color_frame)
# cloud = PyntCloud.from_file("1.ply")
# cloud.plot()
