from typing import Tuple

import numpy as np
import pyrealsense2 as rs2

from amit import clean_up, adjustments


def write_pc_to_file(frame: np.ndarray, adj: Tuple[float, float, float], intrinsics: rs2.intrinsics,
                     filename='test.xyz') -> None:
    assert len(frame.shape) == 2

    with open(filename, 'w') as f:
        for i in range(frame.shape[0]):
            for j in range(frame.shape[1]):
                x, y, z = rs2.rs2_deproject_pixel_to_point(intrinsics, [j, i], frame[i][j])
                f.write('{} {} {}\n'.format(x + adj[0], y + adj[1], z + adj[2]))


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

th_filter = clean_up.get_filter('threshold_filter', *(0.18, 0.22))
temp_filter = clean_up.get_filter('temporal_filter')
hole_filter = clean_up.get_filter('hole_filling_filter')

fil_frame = clean_up.apply_filters(frames, [temp_filter], [hole_filter, th_filter])
mat = np.array(fil_frame.get_data()) / 1000

adj = adjustments.calculate_pc_adjustments_by_frames(og_frame, mat, intrinsics)

print(adj)

num = 1
write_pc_to_file(mat, (0, 0, 0), intrinsics, f'og{num}.xyz')
write_pc_to_file(mat, adj, intrinsics, f'scanned{num}.xyz')
write_pc_to_file(og_frame, (0, 0, 0), intrinsics, f'gen{num}.xyz')
