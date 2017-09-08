#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

#include "Poco/DirectoryIterator.h"
#include "Poco/Path.h"
#include "Poco/File.h"
#include "Poco/Exception.h"
#include "Poco/Net/TCPServer.h"
#include "Poco/Net/TCPServerConnection.h"
#include "Poco/Net/TCPServerConnectionFactory.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/StreamSocket.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "toml.h"

#include <astra/astra.hpp>

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

int frame_number = 0;
int frames_written = 0;
bool quit = false;

std::string config_file;

int capture_num;
bool depth_enable;
bool depth_plot;
bool depth_write;
std::string depth_path;
std::string depth_prefix;
std::string depth_snap_prefix;
Poco::Path d_output_path;

bool rgb_enable;
bool rgb_plot;
bool rgb_write;
std::string rgb_path;
std::string rgb_prefix;
std::string rgb_snap_prefix;
Poco::Path c_output_path;

bool ir_enable;
bool ir_plot;
bool ir_write;
std::string ir_path;
std::string ir_prefix;
std::string ir_snap_prefix;
Poco::Path i_output_path;

bool register_depth;

//bool point_enable;
//bool point_plot;
//bool point_write;
//std::string point_path;
//std::string point_prefix;
//std::string point_snap_prefix;

class ImageConnection: public Poco::Net::TCPServerConnection {
public:
  ImageConnection(const Poco::Net::StreamSocket& s): Poco::Net::TCPServerConnection(s) { }

  void run() {
    Poco::Net::StreamSocket& ss = socket();
    try {
      char buffer[256];
      int n = ss.receiveBytes(buffer, sizeof(buffer));
      while (n > 0) {
        ss.sendBytes(buffer, n);
        n = ss.receiveBytes(buffer, sizeof(buffer));
      }
    }
    catch (Poco::Exception& exc){ 
        std::cerr << "EchoConnection: " << exc.displayText() << std::endl;
    }
  }
};


class DepthFrameListener : public astra::FrameListener {
public:

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override {
        //std::cout << "frame " << frame_number << " ready" << std::endl;
        bool depth_ready = true;
        bool color_ready = true;
        bool ir_ready = true;
        //astra::DepthFrame * depth_frame;
        //astra::ColorFrame * color_frame;

        if(depth_enable) {
             astra::DepthFrame depth_frame = frame.get<astra::DepthFrame>();
            depth_ready = depth_frame.is_valid();
            if(depth_ready) {
                int width = depth_frame.width();
                int height = depth_frame.height();
                memcpy(&depth_images[frame_number][0], depth_frame.data(), width*height*2);
            }

        }

        if(rgb_enable) {
            astra::ColorFrame color_frame = frame.get<astra::ColorFrame>();
            color_ready = color_frame.is_valid();
            if (color_ready) {
                int width = color_frame.width();
                int height = color_frame.height();
                memcpy(&rgb_images[frame_number][0], color_frame.data(), width*height*3);
            }
        }

        if(ir_enable) {
            astra::InfraredFrame16 ir_frame = frame.get<astra::InfraredFrame16>();
            ir_ready = ir_frame.is_valid();
            if (ir_ready) {
                int width = ir_frame.width();
                int height = ir_frame.height();
                memcpy(&ir_images[frame_number][0], ir_frame.data(), width*height*2);
            }
        }

        if (depth_ready && color_ready && ir_ready) {
            frame_number++;
        }
    }
};

void plot_func() {
    //bool quit = false;
    cv::Mat rgb_mat;//(height, width, CV_8UC3);
    cv::Mat depth_mat;//(height, width, CV_16UC1);
    cv::Mat ir_mat;//(height, width, CV_16UC1);
    //std::cerr << "Starting to plot the data..." << std::endl;
    while(!quit) {
        //std::cerr << "plotting...";
        int local_num = frame_number;

        if(local_num < 1) {
            continue;
        }

        if(local_num >= capture_num) {
            break;
        }
        if(depth_enable) {
            //std::cout << "copying depth data in plot thread...";
            depth_mat = cv::Mat(height, width, CV_16UC1);
            memcpy(depth_mat.data, &depth_images[local_num-1][0], width*height*2);
            cv::flip(depth_mat, depth_mat, 1);
            //std::cout << "done!" << std::endl;
        }
        if(rgb_enable) {
            //std::cout << "copying color data in plot thread...";
            rgb_mat = cv::Mat(height, width, CV_8UC3);
            memcpy(rgb_mat.data, &rgb_images[local_num-1][0], width*height*3);
            cv::flip(rgb_mat, rgb_mat, 1);
            //std::cout << "done!" << std::endl;
        }
        if(ir_enable) {
            ir_mat = cv::Mat(height, width, CV_16UC1);
            memcpy(ir_mat.data, &ir_images[local_num-1][0], width*height*2);
            cv::flip(ir_mat, ir_mat, 1);
        }
        if(rgb_plot && rgb_enable) {

            cvtColor(rgb_mat, rgb_mat, cv::COLOR_BGR2RGB);
            cv::imshow("Color", rgb_mat);
        }
        if(depth_plot && depth_enable) {
            cv::Mat depth_plot;
            cv::normalize(depth_mat, depth_plot, 0, 1<<16-1, cv::NORM_MINMAX, CV_16UC1);
            cv::imshow("Depth", depth_plot);
        }
        if(ir_plot && ir_enable) {
            cv::Mat ir_plot;
            cv::normalize(ir_mat, ir_plot, 0, 1<<16-1, cv::NORM_MINMAX, CV_16UC1);
            cv::imshow("IR", ir_plot);
        }
        int key = cv::waitKey(20);
        switch(key) {
            case 27:
                quit = true;
                break;
            case 32:
            //    quit = true;
            //    break;
            //case 49:
                std::string image_suffix = ".png";
                if(depth_enable) {
                    try{
                        Poco::File tmpDir(d_output_path);
                        tmpDir.createDirectories();
                    } catch (Poco::FileException &e) {
                        std::cerr << "Error: Failed to create depth output directory" << std::endl;
                        continue;
                    }

                    //if ( dir_error ) {
                    //    std::cerr << "Error: Failed to create depth output directory" << std::endl;
                    //    continue;
                       //did not successfully create directories
                    //}
                    std::stringstream dss;
                    dss << depth_snap_prefix << std::setw(5) << std::setfill('0') << frames_written << image_suffix;

                    Poco::Path depth_path(d_output_path);
                    depth_path.append(Poco::Path(dss.str()));

                    imwrite(depth_path.toString(), depth_mat);
                }
                if(ir_enable) {
                    try{
                        Poco::File tmpDir(i_output_path);
                        tmpDir.createDirectories();
                    } catch (Poco::FileException &e) {
                        std::cerr << "Error: Failed to create IR output directory" << std::endl;
                        continue;
                    }
            
                    //if ( dir_error ) {
                    //    std::cerr << "Error: Failed to create depth output directory" << std::endl;
                    //    continue;
                       //did not successfully create directories
                    //}
                    std::stringstream iss;
                    iss << ir_snap_prefix << std::setw(5) << std::setfill('0') << frames_written << image_suffix;

                    Poco::Path ir_path(i_output_path);
                    ir_path.append(Poco::Path(iss.str()));

                    imwrite(ir_path.toString(), ir_mat);
                }
                if(rgb_enable) {
                    try{
                        Poco::File tmpDir(c_output_path);
                        tmpDir.createDirectories();
                    } catch (Poco::FileException &e) {
                        std::cerr << "Error: Failed to create color output directory" << std::endl;
                        continue;
                    }
                    std::stringstream css;
                    css << rgb_snap_prefix << std::setw(5) << std::setfill('0') << frames_written << image_suffix;

                    Poco::Path rgb_path(c_output_path);
                    rgb_path.append(Poco::Path(css.str()) );

                    imwrite(rgb_path.toString(), rgb_mat);
                }
                frames_written++;
                continue;
        }
    }
}

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

bool execute() {
    std::thread plot_thread;
    //std::thread c_plot_thread;

    if(depth_plot || rgb_plot || ir_plot) {
        std::cerr << "Launching image plot thread!" << std::endl;
        plot_thread = std::thread(plot_func);
    }
    //if(rgb_plot) {
    //    std::cerr << "Launching color plot thread!" << std::endl;
    //    c_plot_thread = std::thread(color_plot_func);
    //}

    if(depth_write) {
        try{
            Poco::File tmpDir(d_output_path);
            tmpDir.createDirectories();
        } catch (Poco::FileException &e) {
            std::cerr << "Error: Failed to create depth output directory" << std::endl;
            return false;
        }
    }

    if(rgb_write) {
        try{
            Poco::File tmpDir(c_output_path);
            tmpDir.createDirectories();
        } catch (Poco::FileException &e) {
            std::cerr << "Error: Failed to create color output directory" << std::endl;
            return false;
        }
    }

    std::cerr << "Starting the stream... " << std::endl;
    auto start = steady_clock::now();
    //int itr = 0;
    while (!quit && (frame_number < capture_num)) {
        auto update_check = astra_temp_update();
    }

    auto end = steady_clock::now();
    double elapsedSeconds = ((end - start).count()) * steady_clock::period::num / static_cast<double>(steady_clock::period::den);

    std::cerr << "...and shutting it down. ";

    if(depth_write && depth_enable) {
        std::cerr << "Writing depth frames to disk...";
        for(int i = 0; i < depth_images.size(); i++) {
            cv::Mat depth_mat(height, width, CV_16UC1, depth_images[i].begin());
            std::string name_suffix = ".png";

            std::stringstream ss;
            ss << depth_prefix << std::setw(5) << std::setfill('0') << i << name_suffix;

            Poco::Path img_path(d_output_path);
            img_path.append(Poco::Path(ss.str()));

            imwrite(img_path.toString(), depth_mat);
            //std::cout << "Writing frame " << i + 1 << " ..." << std::endl;
        }
        std::cerr << "done!" << std::endl;
    }

    if(rgb_write && rgb_enable) {
        std::cerr << "Writing color frames to disk...";
        for(int i = 0; i < rgb_images.size(); i++) {
            cv::Mat rgb_mat(height, width, CV_8UC3, rgb_images[i].begin());
            std::string name_suffix = ".png";

            std::stringstream ss;
            ss << rgb_prefix << std::setw(5) << std::setfill('0') << i << name_suffix;

            Poco::Path img_path(c_output_path);
            img_path.append(Poco::Path(ss.str()));

            imwrite(img_path.toString(), rgb_mat);
            //std::cout << "Writing frame " << i + 1 << " ..." << std::endl;
        }
        std::cerr << "done!" << std::endl;
    }

    if(ir_write && ir_enable) {
        std::cerr << "Writing color frames to disk...";
        for(int i = 0; i < ir_images.size(); i++) {
            cv::Mat ir_mat(height, width, CV_16UC1, ir_images[i].begin());
            std::string name_suffix = ".png";

            std::stringstream ss;
            ss << ir_prefix << std::setw(5) << std::setfill('0') << i << name_suffix;

            Poco::Path img_path(i_output_path);
            img_path.append(Poco::Path(ss.str()));

            imwrite(img_path.toString(), ir_mat);
            //std::cout << "Writing frame " << i + 1 << " ..." << std::endl;
        }
        std::cerr << "done!" << std::endl;
    }

    std::cerr << "Average FPS: " << frame_number/elapsedSeconds << std::endl;

    if(depth_plot || rgb_plot || ir_plot) {
        plot_thread.join();
    }
    //if(rgb_plot) {
    //    c_plot_thread.join();
    //}

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
int main (int argc, char* argv[]) {
    if ( argc != 2 ) {// argc should be 2 for correct execution
    // We print argv[0] assuming it is the program name
        std::cerr << "usage: " << argv[0] << " <configuration-file>" << std::endl;
        return -1;
    }
    else {
        config_file = std::string(argv[1]);
    }

    std::cerr << "Loading configuration file...";
    load_config();
    std::cerr << "done!" << std::endl;

    if(!depth_enable && !rgb_enable && !ir_enable) {
        std::cerr << "Error: No streams enabled" << std::endl;
        return EXIT_FAILURE;
    }

    std::cerr << "Initializing camera... ";
    astra::StreamSet stream_set;
    auto init_check = astra::initialize();
    astra::StreamReader reader = stream_set.create_reader();
    if(depth_enable) {
        astra::ImageStreamMode depth_mode;
        depth_mode.set_width(640);
        depth_mode.set_height(480);
        depth_mode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
        depth_mode.set_fps(30);

        reader.stream<astra::DepthStream>().set_mode(depth_mode);
        reader.stream<astra::DepthStream>().start();
        if(register_depth) {
            reader.stream<astra::DepthStream>().enable_registration(true);
        }
        depth_images = std::vector< depth_array >(capture_num);
        d_output_path = Poco::Path(depth_path);
        std::cout << "Allocating space for " << depth_images.size() << " depth images" << std::endl;
    }
    if(rgb_enable) {
        astra::ImageStreamMode rgb_mode;
        rgb_mode.set_width(640);
        rgb_mode.set_height(480);
        rgb_mode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
        rgb_mode.set_fps(30);
        reader.stream<astra::ColorStream>().set_mode(rgb_mode);
        reader.stream<astra::ColorStream>().start();
        rgb_images = std::vector< rgb_array >(capture_num);
        c_output_path = Poco::Path(rgb_path);
        std::cout << "Allocating space for " << rgb_images.size() << " color images" << std::endl;
    }
    if(ir_enable) {
        astra::ImageStreamMode ir_mode;
        ir_mode.set_width(640);
        ir_mode.set_height(480);
        ir_mode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
        ir_mode.set_fps(30);

        reader.stream<astra::InfraredStream>().set_mode(ir_mode);
        reader.stream<astra::InfraredStream>().start();

        ir_images = std::vector< ir_array >(capture_num);
        i_output_path = Poco::Path(ir_path);
        std::cout << "Allocating space for " << ir_images.size() << " IR images" << std::endl;
    }

    DepthFrameListener listener = DepthFrameListener();
    reader.add_listener(listener);
    std::cerr << "done!" << std::endl;


    //std::thread update_thread(astra_temp_update());
    // executing
    try {
           execute ();
    } catch (const std::bad_alloc&) {
        std::cerr << "Bad alloc" << std::endl;
    } catch (const std::exception&) {
        std::cerr << "Exception" << std::endl;
    }
    if(depth_enable) {
        reader.stream<astra::DepthStream>().stop();
    }
    if (rgb_enable) {
        reader.stream<astra::ColorStream>().stop();
    }
    reader.remove_listener(listener);
    astra::terminate();
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char* argv[]) {
    Poco::Net::TCPServer srv(new Poco::Net::TCPServerConnectionFactoryImpl<ImageConnection>());
    std::cout << "host: " << srv.socket().address().toString();
    srv.start();
}