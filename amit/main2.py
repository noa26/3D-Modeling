import pyrealsense2 as rs2

from amit import utils
from amit.data import Data

if __name__ == '__main__':
    d = Data.load_context(rs2.context())
    d.cameras[0].filters.append(['temporal_filter'])
    d.cameras[0].after_filters.append(['threshold_filter', 0.1, 0.7])

    utils.save_file(d.scan_object(), d.filename)
