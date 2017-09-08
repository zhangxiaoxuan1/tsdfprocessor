//include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdio>
#include <vector>
#include <map>
#include <utility>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <array>
#include <mutex>
#include "../include/tinytoml/toml.h"

using std::chrono::steady_clock;

typedef uint8_t u8;
typedef uint16_t u16;

typedef std::array<u16, 307200> depth_array;
typedef std::array<u8, 921600> rgb_array;
typedef std::array<u16, 307200> ir_array;

std::vector< depth_array > depth_images;
std::vector< rgb_array > rgb_images;
std::vector< ir_array > ir_images;
int height = 480;
int width = 640;

int frame_number;
int frames_written = 0;
bool quit = false;

std::string config_file = "../config/config.toml";

int capture_num;
bool depth_enable;
bool depth_plot;
bool depth_write;
std::string depth_path;
std::string depth_prefix;
std::string depth_snap_prefix;

bool rgb_enable;
bool rgb_plot;
bool rgb_write;
std::string rgb_path;
std::string rgb_prefix;
std::string rgb_snap_prefix;

bool ir_enable;
bool ir_plot;
bool ir_write;
std::string ir_path;
std::string ir_prefix;
std::string ir_snap_prefix;
bool register_depth;

bool load_config() {
    std::ifstream input(config_file);

    toml::ParseResult pr = toml::parse(input);

    if (!pr.valid()) {
        std::cerr << "Could not load config file: " << pr.errorReason << std::endl;
        return false;
    }

    // pr.value is the parsed value.
    const toml::Value& v = pr.value;

    capture_num = v.get<int>("settings.frame_number");

    depth_enable = v.get<bool>("depth.enable");
    depth_plot = v.get<bool>("depth.display");
    depth_write = v.get<bool>("depth.export");
    depth_path = v.get<std::string>("depth.directory");
    depth_prefix = v.get<std::string>("depth.export_prefix");
    depth_snap_prefix = v.get<std::string>("depth.snapshot_prefix");

    rgb_enable = v.get<bool>("color.enable");
    rgb_plot = v.get<bool>("color.display");
    rgb_write = v.get<bool>("color.export");
    rgb_path = v.get<std::string>("color.directory");
    rgb_prefix = v.get<std::string>("color.export_prefix");
    rgb_snap_prefix = v.get<std::string>("color.snapshot_prefix");

    ir_enable = v.get<bool>("ir.enable");
    ir_plot = v.get<bool>("ir.display");
    ir_write = v.get<bool>("ir.export");
    ir_path = v.get<std::string>("ir.directory");
    ir_prefix = v.get<std::string>("ir.export_prefix");
    ir_snap_prefix = v.get<std::string>("ir.snapshot_prefix");

    register_depth = v.get<bool>("depth.registered");

    return true;
}


int main () try {
	std::cerr << "Loading configuration file...";
	    load_config();
	    std::cerr << "done!\n" << std::endl;

	    if(!depth_enable && !rgb_enable && !ir_enable) {
	        std::cerr << "Error: No streams enabled" << std::endl;
	        return EXIT_FAILURE;
	    }

	    // Create a context object. This object owns the handles to all connected realsense devices.
	    rs::context ctx;
	    printf("You have %d connected RealSense devices.\n", ctx.get_device_count());
	    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

	    // Access the device
	    rs::device * dev = ctx.get_device(0);
	    printf("Using device 0, an %s\n", dev->get_name());
	    printf("    Serial number: %s\n", dev->get_serial());
	    printf("    Firmware version: %s\n", dev->get_firmware_version());

	    // Configure all streams to run at 640*480 at 60 frames per second
	    if(depth_enable){
	        dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
	    }
	    if(rgb_enable){
	        dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
	    }
	    if(ir_enable){
	        dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
	    }
	    if(!depth_enable && !rgb_enable && !ir_enable){
	    	printf("No stream enabled, program exits automatically.\n");
	    	return EXIT_SUCCESS;
	    }
	    dev->start();

	    // Camera warmup - Dropped several first frames to let auto-exposure stabilize
	    for(int i = 0; i < 30; i++){
	       dev->wait_for_frames();
	    }

	    // Display in a GUI
	    cv::namedWindow("Display Image");

	    for(int i = 0; i < capture_num; i++){
	       cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), cv::Mat::AUTO_STEP);
	       cv::imshow("Display Image", color);
	       int key = cv::waitKey(200);
	       switch(key) {
	       	case 27:
	        	break;
	       }
	    }
	    return EXIT_SUCCESS;

} catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
