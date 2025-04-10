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
                    print(json.dumps(data, indent=2))
                    return data
                except json.JSONDecodeError as e:
                    print("Invalid JSON:", e)


# get lidar scan as dict
def get_lidar_scan():
    frame = get_raw_frame()

    data = {}
    data["header"] = frame["header"]
    data["ver_len"] = frame["ver_len"]
    data["rps"] = frame["speed"] / 360
    data["start_angle"] = frame["start_angle"] / 100
    data["end_angle"] = frame["end_angle"] / 100
    # timestamp: "length 2 Byte, the unit for ms, 30000, 30000 to count;" (??)
    data["timestamp"] = frame["timestamp"]
    data["crc8"] = frame["crc8"]
    data["distances"] = np.zeros(POINT_PER_PACK)
    data["intensities"] = np.zeros(POINT_PER_PACK)
    data["angles"] = np.zeros(POINT_PER_PACK)

    if data["end_angle"] < data["start_angle"]:
        data["end_angle"] += 360
    step = (data["end_angle"] - data["start_angle"]) / POINT_PER_PACK
    for i, point in enumerate(frame["points"]):
        data["distances"][i] = point["distance"]
        data["intensities"][i] = point["intensity"]
        data["angles"][i] = data["start_angle"] + step * i

    return data


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
        lidar_data = get_lidar_scan()

        # if the speed is not ~6 RPS, skip this frame
        radar_speed_deg = get_speed_deg(lidar_data)
        if abs((radar_speed_deg / 360) - 6) > 0.2:
            continue

        # waits until we have a reasonable Hertz approximation
        TIMES.append(time.time())
        if len(TIMES) > 10:
            time_diffs = np.diff(np.array(TIMES[-10:]))
            avg_time_diff = np.mean(time_diffs)
        else:
            continue

        # get distances
        distances = get_distances(lidar_data)

        # get angles
        angles = get_angles(lidar_data)

        # Convert angles from degrees to radians
        angles_radians = np.radians(angles)

        scatter.set_offsets(np.c_[-1 * ANGLES, DISTS])
        if verbose:
            print("-")
            print(f"new data posted at {time.strftime('%H:%M:%S')}")
            print(f"(start angle, stop angle): ({angles[-1]}, {angles[0]})")
            print(f"sweep range: {angles[-1] - angles[0]}")
            print(
                f"RPM: {radar_speed_deg / 360}"
            )  # also fishy, sometimes its significanly > 6
            print(f"angle increment between distances: {angles[1] - angles[0]}")
            if len(TIMES) > 10:
                print(f"average hz over last 10 frames: {1 / avg_time_diff}")

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
            max_dist=4, num_dispay_frames=10, verbose=args.verbose
        )
