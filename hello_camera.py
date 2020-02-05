"""
use python 3.6 only
Run this file to see if everything is downloaded properly
"""

import pyrealsense2 as rs


def fun(d: rs.device):
    print(d.sensors)


def main():
    pipe = None
    try:
        pipe = rs.pipeline()
        profile = pipe.start()  # must be called before "wait_for_frames()"
        print(profile.get_device(), end="\n\n")
        fun(profile.get_device())
    finally:
        pipe.stop()

    print("done")


if __name__ == "__main__":
    main()
