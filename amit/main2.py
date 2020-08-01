# import pyrealsense2 as rs2

from amit import utils
from amit.data import Data


def calibrate(config_name: str):
    with open(config_name) as f:
        d = Data.load_json(f)

    d.scan_and_calibrate_all()

    with open(config_name, 'w') as f:
        d.dump_json(f)


def scan(config_name: str):
    with open(config_name) as f:
        d = Data.load_json(f)

    utils.save_file(d.scan_object(), d.filename)


if __name__ == '__main__':
    scan('config.json')


