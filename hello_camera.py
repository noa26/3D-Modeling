"""
use python 3.6 only
Run this file to see if everything is downloaded properly
"""

import pyrealsense2 as rs
import util


def main():
    pipe = None
    ctx = rs.context()
    print(len(ctx.devices), "devices connected")

    #   printing devices' serial numbers
    for device in ctx.devices:
        print(util.serial_number(device), end=",")
    print()

    #   trying to get frames -> just printing the timestamp
    timestamps = []
    try:
        pipe = rs.pipeline()
        profile = pipe.start()

        # just so you'd know how to get a serial number from the pipe
        print(util.serial_number(profile.get_device()))

        for i in range(10):
            frames = pipe.wait_for_frames(timeout_ms=10000)
            for frame in frames:
                timestamps.append(frame.get_timestamp())

        print("picture were taken between", min(timestamps), "and", max(timestamps))

    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
