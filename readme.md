# Instructions for building & running

Modify lines 15 & 16 of ./src/CMakeLists.txt to point to the headers and lib files of your orbbbec astra driver installation.

Navigate to ./build and type 'cmake ..' followed by 'make'

This will put a binary called 'snapshot' into the ./build/bin directory.

Before running the program, change the config.toml file to your specifications.

When running the program, pass the config file path as an argument.

# Notes

Run: g++ -std=c++11 snapshot_intel.cpp -lrealsense -lopencv_core -lopencv_highgui && ./a.out

memcpy(dest,src,size) where count is the "sizeof" of the array. This is used for array operations for rgb, ir and depth.

RGB has 3 channels each 1 byte, so depth*height*3; IR and Depth has 1 channel each 2 bytes, so depth*height*2

cv::imshow displays the image in a window.

cv::normalize(_src, dst, 0, 255, NORM_MINMAX, CV_8UC1) makes sure min of dst is 0, max of dst is 255. 
min value of dst is alpha and max value of dst is beta

cv::waitKey returns ASCII code of key pressed on the openCV window

plot_func() repeat and print on screen until local_num >= capture_num (How did local_num go up??) or esc key
If space is pressed, write current frame as file???

Why cv::flip??
