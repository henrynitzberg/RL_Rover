#include <iostream>
#include <fstream>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdio>
#include <errno.h>
// sudo apt-get install libjsoncpp-dev
#include <json/json.h>

#define POINT_PER_PACK 12
#define HEADER 0x54

typedef struct attribute ((packed)) { uint16_t distance;
    uint8_t intensity;
} LidarPointStructDef;

typedef struct attribute ((packed)) {
    uint8_t  header;
    uint8_t  ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_ PER_ PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t  crc8;
} LiDARFrame TypeDef;

#define POINT_PER_PACK 12
#define HEADER 0x54

// Function to configure the serial port
int configure_serial_port(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port" << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B230400);  // Baud rate 115200
    cfsetospeed(&options, B230400);  // Baud rate 115200
    options.c_cflag &= ~PARENB;      // No parity
    options.c_cflag &= ~CSTOPB;      // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;          // 8 data bits
    options.c_cflag &= ~CRTSCTS;     // Disable hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// Function to read the data from the serial port
int read_lidar_frame(int fd, LiDARFrameTypeDef& frame) {
    ssize_t n = read(fd, &frame, sizeof(LiDARFrameTypeDef));
    if (n < 0) {
        std::cerr << "Error reading from serial port" << std::endl;
        return -1;
    }
    return n;
}

// Function to convert the LiDARFrame to JSON
Json::Value lidar_frame_to_json(const LiDARFrameTypeDef& frame) {
    Json::Value root;

    root["header"] = frame.header;
    root["ver_len"] = frame.ver_len;
    root["speed"] = frame.speed;
    root["start_angle"] = frame.start_angle;

    Json::Value points(Json::arrayValue);
    for (int i = 0; i < POINT_PER_PACK; i++) {
        Json::Value point;
        point["distance"] = frame.point[i].distance;
        point["intensity"] = frame.point[i].intensity;
        points.append(point);
    }

    root["points"] = points;
    root["end_angle"] = frame.end_angle;
    root["timestamp"] = frame.timestamp;
    root["crc8"] = frame.crc8;

    return root;
}

int main() {
    const char* serial_device = "/dev/ttyUSB0";
    const char* out_file = "./lidar_data_tmp.txt";

    int fd = configure_serial_port(serial_device);
    if (fd == -1) {
        return 1;
    }

    std::ofstream output_file_stream(output_file, std::ios::trunc);
    if (!output_file_stream.is_open()) {
        std::cerr << "Failed to open file for writing: " << output_file << std::endl;
        close(fd);
        return 1;
    }


    LiDARFrameTypeDef frame;
    while (true) {
        if (read_lidar_frame(fd, frame) > 0) {
            Json::Value json_frame = lidar_frame_to_json(frame);
            // print
            std::cout << json_frame.toStyledString() << std::endl;
            // and write to temp file
            output_file_stream << json_frame.toStyledString() << std::endl;
        }
        usleep(100000);  // Sleep for a short time (100ms)
    }
    close(fd);
    output_file_stream.close();

    if (std::remove(output_file) != 0) {
        std::cerr << "Error deleting the file: " << output_file << std::endl;
    } else {
        std::cout << "File deleted successfully: " << output_file << std::endl;
    }
    
    return 0;
}