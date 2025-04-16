import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import time
import json

POINT_PER_PACK = 12

pipe_path = "/tmp/lidar_pipe"
while not os.path.exists(pipe_path):
    print("waiting for pipe to be created")
    time.sleep(1)


def get_raw_frame():
    with open(pipe_path, "r") as fifo:
        while True:
            line = fifo.readline()
            if line:
                try:
                    data = json.loads(line)
                    print("Received LiDAR data:")
                    return data
                except json.JSONDecodeError as e:
                    print("Invalid JSON:", e)


# get lidar scan as dict
def get_lidar_scans(num_scans=1):
    scans = []
    for _ in range(num_scans):
        frame = get_raw_frame()

        data = {}
        data["header"] = frame["header"]
        data["ver_len"] = frame["ver_len"]
        data["rps"] = frame["speed"] / 360
        data["start_angle"] = frame["start_angle"] / 100
        data["end_angle"] = frame["end_angle"] / 100
        # timestamp: "length 2 Byte, the unit for ms, 30000, 30000 to count;" (??)
        # data["timestamp"] = frame["timestamp"]
        data["timestamp"] = time.time()
        data["crc8"] = frame["crc8"]
        data["distances"] = np.zeros(POINT_PER_PACK)  # meters
        data["intensities"] = np.zeros(POINT_PER_PACK)  # ???
        data["angles"] = np.zeros(POINT_PER_PACK)  # degrees

        # temp fix
        if data["end_angle"] < data["start_angle"]:
            data["end_angle"] += 360

        step = (data["end_angle"] - data["start_angle"]) / (POINT_PER_PACK - 1)
        for i, point in enumerate(frame["points"]):
            data["distances"][i] = point["distance"] / 1000  # mm/m
            data["intensities"][i] = point["intensity"]
            data["angles"][i] = data["start_angle"] + step * i
        scans.append(data)

    return scans


def display_live_lidar_reading(max_dist=8, num_display_frames=10, verbose=False):
    DISTS = np.array([])
    ANGLES = np.array([])
    TIMES = []

    # Create the figure and axis for the polar plot
    fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
    ax.set_title("Live LiDAR Sensor Reading", va="bottom")
    scatter = ax.scatter([], [], marker="o", color="b")
    ax.set_ylim(0, max_dist)

    start_angle = 0

    while True:
        lidar_data = get_lidar_scans()[0]

        # if the speed is not ~6 RPS, skip this frame
        radar_rps = lidar_data["rps"]
        if not (5.8 < radar_rps < 6.2):
            if verbose:
                print(f"RPS should be ~6, but is {radar_rps}. Skipping frame.")
            continue

        # get distances and angles
        distances = lidar_data["distances"]
        angles = lidar_data["angles"]

        # Convert angles from degrees to radians
        angles_radians = np.radians(angles)

        DISTS = np.concatenate((DISTS, distances))
        ANGLES = np.concatenate((ANGLES, angles_radians))
        TIMES.append(lidar_data["timestamp"])

        if len(TIMES) > num_display_frames:
            scatter.set_offsets(
                np.c_[
                    -1 * ANGLES[-num_display_frames * POINT_PER_PACK :],
                    DISTS[-num_display_frames * POINT_PER_PACK :],
                ]
            )
        else:
            scatter.set_offsets(np.c_[-1 * ANGLES, DISTS])

        if verbose:
            print("-")
            # print(f"new data posted at {lidar_data["timestamp"].strftime('%H:%M:%S')}")
            print(f"(start angle, stop angle): ({angles[0]}, {angles[-1]})")
            print(f"sweep range: {angles[-1] - angles[0]}")
            print(f"distances: {distances}")
            print(f"angle increment (step): {angles[1] - angles[0]}")
            print(f"RPS: {radar_rps}")

        plt.draw()
        plt.pause(0.01)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="argparser")
    parser.add_argument(
        "-t", "--test", action="store_true", help="display lidar readings as video"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="print on recieve data"
    )
    args = parser.parse_args()

    if args.test:
        display_live_lidar_reading(
            max_dist=1, num_display_frames=18, verbose=args.verbose
        )
