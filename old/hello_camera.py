"""
use python 3.6 only
Run this file to see if everything is downloaded properly
"""

import pyrealsense2 as rs
from old import util


def main():
    pipe = None
    ctx = rs.context()
    print(len(ctx.devices), "devices connected.")

    # getting frames frames and printing timestamp range
    timestamps = []
    if len(ctx.devices):

        #   printing devices' serial numbers
        serial_numbers = [util.serial_number(device) for device in ctx.devices]
        print("devices S/N:", ", ".join(serial_numbers))

        try:
            pipe = rs.pipeline()

            # profile is needed for distinguishing between cameras
            profile = pipe.start()

            for i in range(10):
                frames = pipe.wait_for_frames(timeout_ms=10000)
                for frame in frames:
                    timestamps.append(frame.get_timestamp())

            print("picture were taken between", min(timestamps), "and", max(timestamps))

        finally:
            pipe.stop()
    else:
        print("can't get frames!")


if __name__ == "__main__":
    main()
