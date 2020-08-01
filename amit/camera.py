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
        """
        :param fp: A `.read()`-supporting file-like object containing a JSON document, encoding the camera
        :return: A Camera Object decoded from 'fp' data
        """
        return Camera.loads_json(fp.read())

    @staticmethod
    def loads_json(json_str: str) -> 'Camera':
        """
        :param json_str: A json formatted string encoding the camera
        :return: A Camera Object decoded from the string
        """
        params = json.loads(json_str)
        cam = Camera.load_dict(params)
        return cam

    @staticmethod
    def load_dict(params: Dict[str, Any]) -> 'Camera':
        """
        :param params: A dictionary encoding the camera
        :return: A Camera Object decoded from the Dict
        """
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
        """
        :param serial: The serial number of the camera
        :return: The camera type of the camera: depth or lidar
        :raise: If camera is not found raises and exception
        """
        for device in rs2.context().devices:
            if device.get_info(rs2.camera_info.serial_number) == serial:
                return CameraType(device.get_info(rs2.camera_info.product_line)[0])
        raise Exception(f'The device with serial number {serial} is not connected')

    def to_dict(self: 'Camera') -> Dict[str, Any]:
        """
        :return: A dictionary representing the camera
        """
        return {'serial': self.serial, 'distance': self.distance, 'angle': self.angle, 'dx': self.dx, 'dy': self.dy,
                'dz': self.dz, 'filters': self.filters, 'after_filters': self.after_filters, 'on': self.on}

    def dumps_json(self: 'Camera', indent=2) -> str:
        """
        :param indent: The json indent level
        :return: A json formatted string encoding the camera
        """
        return json.dumps(self.to_dict(), indent=indent)

    def dump_json(self: 'Camera', fp: TextIO, indent=2) -> None:
        """
        Writes a json formatted string encoding the camera to fp.
        :param fp: a `.write()`-supporting file-like object
        :param indent: The json indent level
        :return: None
        """
        fp.write(self.dumps_json(indent))

    def get_conifg(self: 'Camera') -> rs2.config:
        """
        :return: A config for this camera
        """
        config = rs2.config()
        config.enable_stream(rs2.stream.depth)
        config.enable_device(self.serial)
        return config

    @staticmethod
    def capture_frames(config: rs2.config, number_of_frames: int = 1,
                       number_of_dummy_frames: int = 0) -> List[rs2.depth_frame]:
        """
        Capture frames via a certain config.
        :param config: The config to capture the frames from
        :param number_of_frames: The number of frames to capture
        :param number_of_dummy_frames: The number of dummy frames to capture before capturing
        :return: A list of the frames that have been captured
        """
        pipe: rs2.pipeline = rs2.pipeline()

        pipe.start(config)
        try:
            for _ in range(number_of_dummy_frames):
                current_frame = pipe.wait_for_frames()
                while not current_frame:
                    current_frame = pipe.wait_for_frames()

            frames = []
            for _ in range(number_of_frames):
                current_frame = pipe.wait_for_frames()
                while not current_frame:
                    current_frame = pipe.wait_for_frames()
                frames.append(current_frame.get_depth_frame())

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

        temp_filter = rs2.temporal_filter()
        for frame in frames:
            frame = temp_filter.process(frame)

        filters = utils.get_filters(self.filters)
        for fil in filters:
            frame = fil.process(frame)

        return frame

    def generate_adapted_point_cloud(self: 'Camera', number_of_frames: int = 1,
                                     number_of_dummy_frames: int = 0, filter_predicate=lambda p: False) -> np.ndarray:
        """
        Generate a point cloud and adapts it (e.g rotates it, invert, add filters after capturing etc..)
        :param number_of_frames: The number of frames to capture
        :param number_of_dummy_frames: The number of dummy frames to capture before capturing
        :param filter_predicate: A predicate to indicate whether to remove a point or not (if True, it is removed)
        :return: The point cloud generated after all the calculations
        """
        config = self.get_conifg()  # Config has to keep a reference 'cause pyrealsense2 and python are stupid
        return self.frames_to_adapted_point_cloud(
            Camera.capture_frames(config, number_of_frames, number_of_dummy_frames), filter_predicate)

    def frames_to_adapted_point_cloud(self: 'Camera', frames: List[rs2.depth_frame],
                                      filter_predicate=lambda p: False) -> np.ndarray:
        """
        Generates a point cloud and adapts it (see generate_adapted_point_cloud) from a list of frame,
        applying filters on the frames to get one frame and calculating the point cloud.
        :param frames: A list of frames to calculate the point cloud from
        :param filter_predicate:A predicate to indicate whether to remove a point or not (if True, it is removed)
        :return: The point cloud generated after all the calculations
        """
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
