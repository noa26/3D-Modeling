import json
from concurrent.futures.process import ProcessPoolExecutor
from typing import List, Dict, Any, TextIO

import numpy as np
import pyrealsense2 as rs2

from amit.camera import Camera, CameraType


class Data:
    cameras: List[Camera]
    number_of_frames: int
    number_of_dummy_frames: int
    filename: str
    max_workers: int
    debug: bool

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
    def load_dict(cfg_dict: Dict[str, Any]) -> 'Data':
        """
        :param cfg_dict: A dictionary encoding the data
        :return: A Data Object decoded from the Dict
        """
        data = Data()

        data.cameras = []
        for cam_param in cfg_dict['cameras']:
            data.cameras.append(Camera.load_dict(cam_param))

        data.number_of_frames = 15 if 'number_of_frame' not in cfg_dict else cfg_dict['number_of_frame']
        data.number_of_dummy_frames = 30 if 'number_of_dummy_frames' not in cfg_dict else \
            cfg_dict['number_of_dummy_frames']
        data.filename = 'default.stl' if 'filename' not in cfg_dict else cfg_dict['filename']
        data.max_workers = 1 if 'max_workers' not in cfg_dict else cfg_dict['max_workers']
        data.debug = False if 'debug' not in cfg_dict else cfg_dict['debug']

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

    def to_dict(self:'Data') -> Dict[str, Any]:
        """
        :return: A dictionary representing the data
        """
        dict = {'number_of_frames': self.number_of_frames, 'number_of_dummy_frames': self.number_of_dummy_frames,
                   'filename': self.filename, 'debug': self.debug,
                   'cameras': []}
        for cam in self.cameras:
            dict['cameras'].append(cam.to_dict())
        
        return dict

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
