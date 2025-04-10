#include <cstdio>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
// sudo apt-get install libjsoncpp-dev
#include <json/json.h>

#define POINT_PER_PACK 12
#define HEADER 0x54
#define FRAME_SIZE sizeof(LiDARFrameTypeDef)

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

// Function to configure the serial port
int configure_serial_port(const char *device) {
  int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    std::cerr << "Failed to open serial port. Is the LiDAR sensor plugged in "
                 "and available?"
              << std::endl;
    return -1;
  }

  struct termios options;
  tcgetattr(fd, &options);
  cfsetispeed(&options, B230400);
  cfsetospeed(&options, B230400);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag |= CREAD | CLOCAL;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);

  return fd;
}

// Function to read the data from the serial port
int read_lidar_frame(int fd, LiDARFrameTypeDef &frame) {
  uint8_t buffer[256];
  size_t bytes_read = read(fd, buffer, sizeof(buffer));
  if (bytes_read < FRAME_SIZE)
    return false;

  for (size_t i = 0; i <= bytes_read - FRAME_SIZE; i++) {
    if (buffer[i] == HEADER) {
      std::memcpy(&frame, &buffer[i], FRAME_SIZE);
      return true;
    }
  }
  return false;
}

// Function to convert the LiDARFrame to JSON
Json::Value lidar_frame_to_json(const LiDARFrameTypeDef &frame) {
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
  const char *serial_device = "/dev/ttyUSB0";
  const char *out_stream = "/tmp/lidar_pipe";

  int fd = configure_serial_port(serial_device);
  if (fd == -1) {
    return 1;
  }

  if (access(out_stream, F_OK) == -1) {
    if (mkfifo(out_stream, 0666) != 0) {
      std::cerr << "Failed to create FIFO pipe: " << strerror(errno)
                << std::endl;
      return 1;
    }
  }

  std::ofstream output(out_stream);
  if (!output.is_open()) {
    std::cerr << "Failed to open pipe for writing: " << out_stream << std::endl;
    close(fd);
    return 1;
  }

  LiDARFrameTypeDef frame;
  while (true) {
    if (read_lidar_frame(fd, frame)) {
      Json::Value json_frame = lidar_frame_to_json(frame);
      output << json_frame.toStyledString() << std::endl;
      output.flush();
    }
    usleep(100000); // Sleep for a short time (100ms)
  }
  close(fd);
  output.close();

  return 0;
}