# Requirements

You should have openCV and librealsense installed on your system.

# Instructions for building & running

Navigate to ./build and type 'cmake ..' followed by 'make'

This will put a binary called 'snapshot' into the ./build/bin directory.

Before running the program, change the config.toml file to your specifications.

When running the program, pass the config file path as an argument.

# Notes

Running the cpp file without cmake: g++ -std=c++11 snapshot_intel.cpp -lrealsense -lopencv_core -lopencv_highgui && ./a.out
