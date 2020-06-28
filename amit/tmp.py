import numpy as np
import pyrealsense2 as rs2

from amit import clean_up, adjustments

pipe = rs2.pipeline()
try:
    profile = pipe.start(rs2.config())
    intrinsics = profile.get_stream(rs2.stream.depth).as_video_stream_profile().get_intrinsics()
finally:
    pipe.stop()

og_frame = adjustments.generate_frame_flat_surface(0.09, 0.14, 0.2, intrinsics)
config = rs2.config()
config.enable_stream(rs2.stream.depth)
frames = clean_up.capture_frames(config, 10, 30)

th_filter = clean_up.get_filter('threshold_filter', *(0.19, 0.21))
temp_filter = clean_up.get_filter('temporal_filter')
hole_filter = clean_up.get_filter('hole_filling_filter')

fil_frame = clean_up.apply_filters(frames, [temp_filter], [hole_filter, th_filter])
mat = np.array(fil_frame.get_data()) / 1000

adj = adjustments.calculate_pc_adjustments_by_frames(og_frame, mat, intrinsics)

print(adj)

