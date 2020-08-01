import json
from concurrent.futures.process import ProcessPoolExecutor
from typing import List, Dict, Any, TextIO, Tuple

import numpy as np
import pyrealsense2 as rs2

from amit.camera import Camera, CameraType

# TODO: Add debug staff

class Data:

    def __init__(self: 'Data'):
        self.cameras: List[Camera] = []
        self.number_of_frames: int = 0
        self.number_of_dummy_frames: int = 0
        self.filename: str = ''
        self.max_workers: int = 0
        self.calibration_surface: Tuple[float, float, float] = (0, 0, 0)
        self.debug: bool = False

    @staticmethod
    def load_json(fp: TextIO) -> 'Data':
        """
        :param fp: A `.read()`-supporting file-like object containing a JSON document, encoding the data
        :return: A Data Object decoded from 'fp' data
        """
        return Data.loads_json(fp.read())

    @staticmethod
    def loads_json(json_str: str) -> 'Data':
        """
        :param json_str: A json formatted string encoding the data
        :return: A Data Object decoded from the string
        """
        params = json.loads(json_str)
        data = Data.load_dict(params)
        return data

    @staticmethod
    def load_dict(data_dict: Dict[str, Any]) -> 'Data':
        """
        :param data_dict: A dictionary encoding the data
        :return: A Data Object decoded from the Dict
        """
        data = Data()

        data.cameras = []
        for cam_param in data_dict['cameras']:
            data.cameras.append(Camera.load_dict(cam_param))

        data.number_of_frames = 15 if 'number_of_frame' not in data_dict else data_dict['number_of_frame']
        data.number_of_dummy_frames = 30 if 'number_of_dummy_frames' not in data_dict else \
            data_dict['number_of_dummy_frames']
        data.filename = 'default.stl' if 'filename' not in data_dict else data_dict['filename']
        data.max_workers = 1 if 'max_workers' not in data_dict else data_dict['max_workers']
        data.calibration_surface = (0, 0, 0) if 'calibration_surface' not in data_dict else data_dict['calibration_surface']
        data.debug = False if 'debug' not in data_dict else data_dict['debug']

        return data

    @staticmethod
    def load_context(ctx: rs2.context) -> 'Data':
        """
        :param ctx: A context to load all the cameras from
        :return: A Data Object with default values and all the cameras available in ctx
        """
        cfg_map = {'cameras': []}

        for device in ctx.devices:
            cfg_map['cameras'].append({'serial': device.get_info(rs2.camera_info.serial_number)})

        return Data.load_dict(cfg_map)

    def to_dict(self: 'Data') -> Dict[str, Any]:
        """
        :return: A dictionary representing the data
        """
        data_dict = {'number_of_frames': self.number_of_frames, 'number_of_dummy_frames': self.number_of_dummy_frames,
                     'filename': self.filename, 'debug': self.debug, 'calibration_surface': self.calibration_surface,
                     'cameras': []}
        for cam in self.cameras:
            data_dict['cameras'].append(cam.to_dict())

        return data_dict

    def dumps_json(self: 'Data', indent=2) -> str:
        """
        :param indent: The json indent level
        :return: A json formatted string encoding the data
        """
        return json.dumps(self.to_dict(), indent=indent)

    def dump_json(self: 'Data', fp: TextIO, indent=2) -> None:
        """
        Writes a json formatted string encoding the data to fp.
        :param fp: a `.write()`-supporting file-like object
        :param indent: The json indent level
        :return: None
        """
        fp.write(self.dumps_json(indent))

    def scan_object(self: 'Data') -> np.ndarray:
        """
        :return: A point representing an object captured from all the cameras that are on
        """
        depth_cams = [cam for cam in self.cameras if cam.on and cam.type is CameraType.Depth]
        lidar_cams = [cam for cam in self.cameras if cam.on and cam.type is CameraType.LiDAR]

        # Using processes means effectively side-stepping the Global Interpreter Lock
        with ProcessPoolExecutor(max_workers=min(self.max_workers, len(depth_cams) + len(lidar_cams))) as pool:
            futures = [
                pool.submit(dcam.generate_adapted_point_cloud, self.number_of_frames, self.number_of_dummy_frames)
                for dcam in depth_cams
            ]

            # LiDAR cameras cannot capture simultaneously, for now run them serially
            # TODO: defeat python's shitty concurrency model and make it more concurrent
            pcs = [lcam.generate_adapted_point_cloud(self.number_of_frames, self.number_of_dummy_frames)
                   for lcam in lidar_cams]
            for fut in futures:
                pcs.append(fut.result())

        return np.concatenate(pcs)

    def scan_and_calibrate_all(self: 'Data') -> None:
        """
        Scans a calibration object and adjust the cameras accordingly
        :return: None
        """
        depth_cams = [cam for cam in self.cameras if cam.on and cam.type is CameraType.Depth]
        lidar_cams = [cam for cam in self.cameras if cam.on and cam.type is CameraType.LiDAR]

        # Using processes means effectively side-stepping the Global Interpreter Lock
        with ProcessPoolExecutor(max_workers=min(self.max_workers, len(depth_cams) + len(lidar_cams))) as pool:
            futures = [
                pool.submit(dcam.scan_and_calibrate, self.calibration_surface, self.number_of_frames,
                            self.number_of_dummy_frames)
                for dcam in depth_cams
            ]

            # LiDAR cameras cannot capture simultaneously, for now run them serially
            # TODO: defeat python's shitty concurrency model and make it more concurrent
            _ = [lcam.scan_and_calibrate(self.calibration_surface, self.number_of_frames, self.number_of_dummy_frames)
                 for lcam in lidar_cams]
            for fut in futures:
                fut.result()

        return
