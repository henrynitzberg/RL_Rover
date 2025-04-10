# Compiler
CXX = g++
CXXFLAGS = -Wall -std=c++11 $(shell pkg-config --cflags jsoncpp)
LDFLAGS = $(shell pkg-config --libs jsoncpp)

# Target executable
TARGET = lidar_reader

# Source file
SRC = Lidar_Reader.cpp
OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJ) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(OBJ) $(TARGET)