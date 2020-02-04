"""
use python 3.6 only
Run this file to see if everything is downloaded properly
"""
import pyrealsense2 as rs


def main():
    pipe = rs.pipeline()
    profile = pipe.start()
    print(profile.get_device(), end="\n\n")

    try:
        for i in range(10):
            frames = pipe.wait_for_frames()
            for f in frames:
                print(f.profile)
            print("-" * 70)

    finally:
        pipe.stop()

    print("done")


if __name__ == "__main__":
    main()
