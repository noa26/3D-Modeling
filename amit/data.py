import json
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
        return Data.loads_json(fp.read())

    @staticmethod
    def loads_json(json_str: str) -> 'Data':
        params = json.loads(json_str)
        data = Data.load_map(params)
        return data

    @staticmethod
    def load_map(cfg_map: Dict[str, Any]) -> 'Data':
        data = Data()

        data.cameras = []
        for cam_param in cfg_map['cameras']:
            data.cameras.append(Camera.load_dict(cam_param))

        data.number_of_frames = 15 if 'number_of_frame' not in cfg_map else cfg_map['number_of_frame']
        data.number_of_dummy_frames = 30 if 'number_of_dummy_frames' not in cfg_map else \
            cfg_map['number_of_dummy_frames']
        data.filename = 'default.stl' if 'filename' not in cfg_map else cfg_map['filename']
        data.max_workers = 1 if 'max_workers' not in cfg_map else cfg_map['max_workers']
        data.debug = False if 'debug' not in cfg_map else cfg_map['debug']

        return data

    @staticmethod
    def load_context(ctx: rs2.context) -> 'Data':
        cfg_map = {'cameras': []}

        for device in ctx.devices:
            cfg_map['cameras'].append({'serial': device.get_info(rs2.camera_info.serial_number)})

        return Data.load_map(cfg_map)

    def dumps_json(self: 'Data', indent=2) -> str:
        cfg_map = {'number_of_frames': self.number_of_frames, 'number_of_dummy_frames': self.number_of_dummy_frames,
                   'filename': self.filename, 'debug': self.debug,
                   'cameras': []}
        for cam in self.cameras:
            cfg_map['cameras'].append(cam.dumps_json(indent))
        return json.dumps(cfg_map)

    def dump_json(self: 'Data', fp: TextIO, indent=2) -> None:
        fp.write(self.dumps_json(indent))

    def scan_object(self: 'Data') -> np.ndarray:
        depth_cams = [cam for cam in self.cameras if cam.on and cam.type is CameraType.Depth]
        lidar_cams = [cam for cam in self.cameras if cam.on and cam.type is CameraType.LiDAR]

        # Using processes means effectively side-stepping the Global Interpreter Lock
        # with ThreadPoolExecutor(max_workers=min(self.max_workers, len(depth_cams) + len(lidar_cams))) as pool:
        #     futures = [
        #         pool.submit(dcam.generate_adapted_point_cloud, self.number_of_frames, self.number_of_dummy_frames)
        #         for dcam in depth_cams
        #     ]
        #
        #     # LiDAR cameras cannot capture simultaneously
        #     frames = [lcam.capture_frames(self.number_of_frames, self.number_of_dummy_frames) for lcam in lidar_cams]
        #     futures.extend([
        #         pool.submit(lcam.frames_to_adapted_point_cloud, frames[i])
        #         for i, lcam in enumerate(lidar_cams)
        #     ])
        #
        #     pcs = []
        #     for fut in futures:
        #         pcs.append(fut.result())
        pcs = [dcam.generate_adapted_point_cloud(self.number_of_frames, self.number_of_dummy_frames)
               for dcam in depth_cams] + \
              [lcam.generate_adapted_point_cloud(self.number_of_frames, self.number_of_dummy_frames)
               for lcam in lidar_cams]

        return np.concatenate(pcs)
