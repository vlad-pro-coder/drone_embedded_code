# Output binary name
TARGET := my_program

# Grab all .cpp files recursively
SRCS := $(shell find . -name '*.cpp')

# Virtualenv path for runtime Python (for modules like board/busio)
PYTHON_VENV := ../pythonTest/env/bin/python3

# Include directories
PYBIND11_INC := $(shell $(PYTHON_VENV) -m pybind11 --includes)
PYTHON_INCLUDES := $(shell python3-config --includes)  # system Python headers
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)

INCLUDES := $(PYBIND11_INC) $(PYTHON_INCLUDES) $(OPENCV_CFLAGS)

# Link libraries
PYTHON_LIBS := $(shell python3-config --ldflags) -lpython3.11
OPENCV_LIBS := $(shell pkg-config --libs opencv4)

# Add Pigpio and threading libraries for Raspberry Pi
EXTRA_LIBS := -lpigpio -lrt -pthread

# Compiler flags
CXX := g++
CXXFLAGS := -Wall -Wextra -std=c++17

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SRCS) $(PYTHON_LIBS) $(OPENCV_LIBS) $(EXTRA_LIBS)

clean:
	rm -f $(TARGET)
