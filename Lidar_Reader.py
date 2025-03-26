import struct
import serial
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import time

# #define POINT_PER_PACK 12
# #define HEADER 0x54

# typedef struct   attribute ((packed)) { uint16_t distance;
# uint8_t intensity;

# } LidarPointStructDef;

# typedef struct   attribute ((packed)) {
# uint8_ t  header;
# uint8_ t  ver_ len;
# uint16_ t  speed;
# uint16 t   start angle;
# LidarPointStructDef point[POINT_ PER_ PACK];
# uint16_ t   end_ angle;
# uint16_ t   timestamp;
# uint8_ t   crc8;
# )LiDARFrame TypeDef;

# in the python struct lib: 
# B = 1 byte, unsigned char
# H = 2 bytes, unsigned short

POINTS_PER_PACK = 12
HEADER = 0x54

# Define the format for the Lidar point data
LidarPointStructDef = ' H B'  # uint16_t distance (2 bytes), uint8_t intensity (1 byte)

# Define the format for the full LIDAR frame
LiDARFrameTypeDef = 'B B H H' + LidarPointStructDef * POINTS_PER_PACK + ' H H B'

LiDARFrameSize = struct.calcsize(LiDARFrameTypeDef)

def parse_lidar_frame(packed_data):
    unpacked_data = struct.unpack(LiDARFrameTypeDef, packed_data)

    header = unpacked_data[0]
    ver_len = unpacked_data[1]
    speed = unpacked_data[2]
    start_angle = unpacked_data[3]
    points = np.zeros((POINTS_PER_PACK, 2))
    for i in range(POINTS_PER_PACK):
        distance = unpacked_data[4 + (i * 2)]  # 2 bytes for distance
        intensity = unpacked_data[5 + (i * 2)]  # 1 byte for intensity
        points[i] = (distance, intensity)
    end_angle = unpacked_data[4 + POINTS_PER_PACK * 2]
    timestamp = unpacked_data[5 + POINTS_PER_PACK * 2]
    crc8 = unpacked_data[6 + POINTS_PER_PACK * 2]

    return {
        'header': header,
        'ver_len': ver_len,
        'speed': speed,
        'start_angle': start_angle,
        'points': points,
        'end_angle': end_angle,
        'timestamp': timestamp,
        'crc8': crc8
    }


# if this line is causing trouble, try 'sudo chmod a+rw /dev/ttyUSB0'
ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)

# waits until next frame is available
def get_next_frame():
    while True:
        if ser.in_waiting >= LiDARFrameSize:
            frame = ser.read(LiDARFrameSize)
            if frame[0] == HEADER:
                return parse_lidar_frame(frame)

def display_live_lidar_reading(max_dist=800, static=False, verbose=False):

    DISTS = np.array([])
    ANGLES = np.array([])
    TIMES = []

    # Create the figure and axis for the polar plot
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_title("Live LiDAR Sensor Reading", va='bottom')
    scatter = ax.scatter([], [], marker='o', color='b')
    ax.set_ylim(0, max_dist) 

    start_angle = 0

    while True:
        lidar_data = get_next_frame()

        # if the speed is not 6 RPM, skip this frame
        radar_speed_deg = lidar_data['speed']
        if abs((radar_speed_deg / 360) - 6) > .2:
            continue
        
        # waits until we have a reasonable Hertz approximation
        TIMES.append(time.time())
        if len(TIMES) > 10:
            time_diffs = np.diff(np.array(TIMES[-10:]))
            avg_time_diff = np.mean(time_diffs)
        else:
            continue

        start_angle = lidar_data['start_angle'] / 100
        end_angle = lidar_data['end_angle'] / 100
        # end_angle = avg_time_diff * radar_speed_deg + start_angle
        # maybe? start end angles are looking very suspicious
        if start_angle > end_angle:
            end_angle += 360
        elif end_angle - start_angle < 200:
            end_angle += 360
        step = (end_angle - start_angle) / (POINTS_PER_PACK - 1)

        # Adjust angles if necessary
        if start_angle > end_angle:
            end_angle += 360
        elif end_angle - start_angle < 200:
            end_angle += 360
        step = (end_angle - start_angle) / (POINTS_PER_PACK - 1)

        # # Calculate start and end angles based on previous measurement time and rotational speed
        # time_diff = TIMES[-1] - TIMES[-2]
        # print(time_diff)
        # start_angle = (start_angle + time_diff * radar_speed_deg) % 360
        # end_angle = start_angle + (radar_speed_deg) * time_diff
        # step = (end_angle - start_angle) / (POINTS_PER_PACK- 1)


        distances = lidar_data['points'][:, 0]
        angles = np.zeros(POINTS_PER_PACK)
        for i in range(POINTS_PER_PACK):
            angles[i] = start_angle + i * step


        # Convert angles from degrees to radians
        angles_radians = np.radians(angles)
        if static:
            DISTS = np.concatenate((DISTS, distances), axis=0)
            ANGLES = np.concatenate((ANGLES, angles_radians), axis=0)
        else:
            DISTS = distances
            ANGLES = angles_radians

        scatter.set_offsets(np.c_[-1 * ANGLES, DISTS])
        if verbose:
            print('-')
            print(f"new data posted at {time.strftime('%H:%M:%S')}")
            print(f"(start angle, stop angle): ({start_angle}, {end_angle})")
            print(f"sweep range: {end_angle - start_angle}")
            print(f"RPM: {radar_speed_deg / 360}") # also fishy, sometimes its significanly > 6
            print(f"angle increment between distances: {step}")
            if len(TIMES) > 10:
                print(f"average hz over last 10 frames: {1 / avg_time_diff}")


        plt.draw()
        plt.pause(0.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="argparser")
    parser.add_argument('-t', '--test', action='store_true', help="display lidar readings as video")
    parser.add_argument('-v', '--verbose', action='store_true', help="print on recieve data")
    args = parser.parse_args()

    if args.test:
        display_live_lidar_reading(max_dist=400, static=True, verbose=args.verbose)