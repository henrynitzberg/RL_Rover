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

    # Create the figure and axis for the polar plot
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_title("Live LiDAR Sensor Reading", va='bottom')
    scatter = ax.scatter([], [], marker='o', color='b')
    ax.set_ylim(0, max_dist) 

    while True:
        lidar_data = get_next_frame()

        start_angle = lidar_data['start_angle'] / 100
        end_angle = lidar_data['end_angle'] / 100\
        # maybe? start end angles are looking very suspicious
        if start_angle > end_angle:
            end_angle += 360

        step = (end_angle - start_angle) / (POINTS_PER_PACK - 1)

        distances = lidar_data['points'][:, 0]
        angles = np.zeros(POINTS_PER_PACK)
        for i in range(POINTS_PER_PACK):
            angles[i] = start_angle + i
        

        # Convert angles from degrees to radians
        angles_radians = np.radians(angles)
        if static:
            DISTS = np.concatenate((DISTS, distances), axis=0)
            ANGLES = np.concatenate((ANGLES, angles_radians), axis=0)
        else:
            DISTS = distances
            ANGLES = angles_radians

        scatter.set_offsets(np.c_[ANGLES, DISTS])
        if verbose:
            print('-')
            print(f"new data posted at {time.strftime('%H:%M:%S')}")
            print(f"(start angle, stop angle): ({start_angle}, {end_angle})")
            print(f"sweep range: {end_angle - start_angle}")


        plt.draw()
        plt.pause(0.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="argparser")
    parser.add_argument('-t', '--test', action='store_true', help="display lidar readings as video")
    parser.add_argument('-v', '--verbose', action='store_true', help="print on recieve data")
    args = parser.parse_args()

    if args.test:
        display_live_lidar_reading(max_dist=800, static=True, verbose=args.verbose)