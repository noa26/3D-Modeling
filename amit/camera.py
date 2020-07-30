import json
from enum import Enum
from typing import List, Any, Dict, Iterable, TextIO

import numpy as np
import pyrealsense2 as rs2

from amit import utils


class CameraType(Enum):
    Depth = 'D'
    LiDAR = 'L'


class Camera:
    serial: str
    distance: float
    angle: float
    type: CameraType
    dx: float
    dy: float
    dz: float
    filters: List[List[Any]]
    after_filters: List[List[Any]]
    on: bool

    @staticmethod
    def load_json(fp: TextIO) -> 'Camera':
        return Camera.loads_json(fp.read())

    @staticmethod
    def loads_json(json_str: str) -> 'Camera':
        params = json.loads(json_str)
        cam = Camera.load_dict(params)
        return cam

    @staticmethod
    def load_dict(params: Dict[str, Any]) -> 'Camera':
        cam = Camera()
        cam.serial = params['serial']
        cam.type = Camera.get_camera_type(cam.serial)
        cam.angle = 0 if 'angle' not in params else params['angle']
        cam.distance = 0 if 'distance' not in params else params['distance']
        cam.dx = 0 if 'dx' not in params else params['dx']
        cam.dy = 0 if 'dy' not in params else params['dy']
        cam.dz = 0 if 'dz' not in params else params['dz']
        cam.filters = [] if 'filters' not in params else params['filters']
        cam.after_filters = [] if 'after_filters' not in params else params['after_filters']
        cam.on = True if 'on' not in params else params['on']
        return cam

    @staticmethod
    def get_camera_type(serial: str) -> CameraType:
        for device in rs2.context().devices:
            if device.get_info(rs2.camera_info.serial_number) == serial:
                return CameraType(device.get_info(rs2.camera_info.product_line)[0])
        raise Exception(f'The device with serial number {serial} is not connected')

    def dumps_json(self: 'Camera', indent=2) -> str:
        params = {'serial': self.serial, 'distance': self.distance, 'angle': self.angle, 'dx': self.dx, 'dy': self.dy,
                  'dz': self.dz, 'filters': self.filters, 'after_filters': self.after_filters, 'on': self.on}
        return json.dumps(params, indent=indent)

    def dump_json(self: 'Camera', fp: TextIO, indent=2) -> None:
        fp.write(self.dumps_json(indent))

    def capture_frames(self: 'Camera', number_of_frames: int = 1, number_of_dummy_frames: int = 0) -> \
            List[rs2.depth_frame]:
        """
        Capture frames via a certain config.
        :param number_of_frames: The number of frames to capture
        :param number_of_dummy_frames: The number of dummy frames to capture before capturing (If that makes sense lol)
        :return: A list of the frames that have been captured
        """

        config: rs2.config = rs2.config()
        config.enable_stream(rs2.stream.depth)
        config.enable_device(self.serial)

        pipe: rs2.pipeline = rs2.pipeline()

        pipe.start(config)
        try:
            for _ in range(number_of_dummy_frames):
                pipe.wait_for_frames()

            frames = []
            for _ in range(number_of_frames):
                frames.append(pipe.wait_for_frames().get_depth_frame())

            return frames
        finally:
            pipe.stop()

    def apply_filters(self: 'Camera', frames: Iterable[rs2.depth_frame]) -> rs2.frame:
        """
        Apply a set of filters on the frames that were captured.
        :param frames: The frames to filter
        :return: The final frame after adding filters
        """

        frame: rs2.frame = frames[0]

        filters = utils.get_filters(self.filters)
        for frame in frames:
            for fil in filters:
                frame = fil.process(frame)

        filters = utils.get_filters(self.after_filters)
        for fil in filters:
            frame = fil.process(frame)

        return frame

    def generate_adapted_point_cloud(self: 'Camera', number_of_frames: int = 1,
                                     number_of_dummy_frames: int = 0, filter_predicate=lambda p: False) -> np.ndarray:

        return self.frames_to_adapted_point_cloud(
            self.capture_frames(number_of_frames, number_of_dummy_frames), filter_predicate)

    def frames_to_adapted_point_cloud(self: 'Camera', frames: List[rs2.depth_frame],
                                      filter_predicate=lambda p: False) -> np.ndarray:

        frame = self.apply_filters(frames)
        pc: rs2.pointcloud = rs2.pointcloud()
        points: np.ndarray = np.array(pc.calculate(frame).get_vertices())
        # Convert to viable np.ndarray, and also filter 'zero' points and accounts for deviation
        points = np.array([(x + self.dx, y + self.dy, z + self.dz) for (x, y, z) in points if not x == y == z == 0])

        # Invert x, y axis and mirror the z axis around the distance line
        utils.change_coordinates_inplace(points,
                                         lambda p: (-p[0], -p[1], self.distance - p[2]))

        points = utils.filter_points(points, filter_predicate)
        utils.rotate_point_cloud_inplace(points, self.angle)
        return points
