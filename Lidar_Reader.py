import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt

DISTS = np.array([])
ANGLES = np.array([])

# Define constants based on your provided data structure
POINTS_PER_PACK = 12
HEADER = 0x54

# Define the format for the Lidar point data
LIDAR_POINT_STRUCT = ' H B'  # uint16_t distance (2 bytes), uint8_t intensity (1 byte)

#B = 1 byte, H = 2 bytes

# Define the format for the full LIDAR frame
LIDAR_FRAME_FORMAT = 'B B H H' + LIDAR_POINT_STRUCT * POINTS_PER_PACK + ' H H B'
LIDAR_FRAME_SIZE = struct.calcsize(LIDAR_FRAME_FORMAT)

baudrates = [4800, 9600, 19200, 38400, 57600, 115200, 230400]

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', baudrates[-1], timeout=1)
#230400
def parse_lidar_frame(frame):
    """ Parse a raw Lidar frame into its respective components. """
    unpacked_data = struct.unpack(LIDAR_FRAME_FORMAT, frame)

    # Header
    header = unpacked_data[0]
    # Version/Length
    ver_len = unpacked_data[1]
    # Speed
    speed = unpacked_data[2]
    # Start angle
    start_angle = unpacked_data[3]
    # Lidar points
    points = []
    for i in range(POINTS_PER_PACK):
        distance = unpacked_data[4 + i * 2]  # 2 bytes for distance
        intensity = unpacked_data[5 + i * 2]  # 1 byte for intensity
        if (intensity >= 200):
            points.append((distance, intensity))
        else:
            points.append((0, intensity))
    # End angle
    end_angle = unpacked_data[4 + POINTS_PER_PACK * 2]
    # Timestamp
    timestamp = unpacked_data[5 + POINTS_PER_PACK * 2]
    # CRC8
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

def get_lidar():

    """ Continuously read data from the Lidar sensor. """
    while True:
        if ser.in_waiting >= LIDAR_FRAME_SIZE:
            frame = ser.read(LIDAR_FRAME_SIZE)
            # Check header byte
            if frame[0] == HEADER:
                lidar_data = parse_lidar_frame(frame)
                return lidar_data
            else:
                pass
                #print("Invalid header byte, skipping frame")
                #lidar_data = parse_lidar_frame(frame)
                #print(lidar_data)
                #time.sleep(5)
        else:
            #pass
            time.sleep(0.1)
            #print("Waiting for more data...")

    
def display_live_lidar_reading():

    DISTS = np.array([])
    ANGLES = np.array([])
    # Number of distances
    num_readings = 12
    
    # Generate initial angles based on the number of readings
    #angles = np.linspace(start_angle, end_angle, num_readings)
    
    # Create the figure and axis for the polar plot
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_title("Live LiDAR Sensor Reading", va='bottom')

    # Use scatter instead of plot
    scatter = ax.scatter([], [], marker='o', color='b')

    # Set the radial limits (distance range)
    ax.set_ylim(0, 1000)  # Adjust this according to your expected distance range
    
    # Loop to update the plot in real-time
    while True:
        # Get new LiDAR readings
        lidar_data = get_lidar()

        start_angle = lidar_data['start_angle'] / 100
        end_angle = lidar_data['end_angle'] / 100

        distances = np.array(lidar_data['points'])[:, 0]


        print(lidar_data['points'])
        #print(distances)

        #double check and probably fix
        angles = np.linspace(start_angle, end_angle, 12)
        step = (end_angle - start_angle)/(POINTS_PER_PACK - 1)
        angles = []
        for i in range(POINTS_PER_PACK):
            angles.append(start_angle + step*i)

        print('start angle, end angle, step:', start_angle, end_angle, step)

        # Convert angles from degrees to radians
        angles_radians = np.radians(angles)

        DISTS = np.concatenate((DISTS, distances), axis=0)
        ANGLES = np.concatenate((ANGLES, angles_radians), axis=0)
        # Update the plot with new data
        #line.set_data(angles_radians, distances)
        scatter.set_offsets(np.column_stack((-1 * ANGLES, DISTS)))
        # Redraw the plot
        plt.draw()
        
        # Pause to simulate real-time data fetching (adjust as needed)
        plt.pause(0.1)  # 0.1 seconds delay between updates

if __name__ == '__main__':
    display_live_lidar_reading()
    #read_lidar_data()