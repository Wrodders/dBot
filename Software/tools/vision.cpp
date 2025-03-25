/************************************************************************************
 *  @file    vision.cpp
 *  @brief   Monocular Vision Navigation
 * 
 * Gstreamer Pipeline:
 * Reads frames in YUV from gstreamer appsrc and writes to appsink
 * For each captured frame generates a continuous trajectory as a set of linear and angular velocity commands
 * Performs interactive calibration of the camera using OpenCV and saves the calibration parameters to a file
 * Publishes the processed frames to the output pipe for visualization
 * 
 * 
 * Command Server 
 * Implements a pub-rpc client over ZMQ, Listens for commands on VISION topic over IPC; 
 * these modify runtime parameters through config managers
 * Outputs CMD_RET/VISION messages to the message socket
 * 
 * Target Frame Rate: 30 Hz
 * Output frame for visualization
 * 
 *         640px           
 * |---------------------|
 * | color| Birds| Hist  | 
 * |------| -----|-------| 480px
 * | ROI  | Edge | window|
 * ----------------------|
 ************************************************************************************/
#include <gst/gst.h>
#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp> 

#ifdef __APPLE__
#include <TargetConditionals.h>
    #define DEFAULT_INPUT_PIPELINE "avfvideosrc device-index=0 do-timestamp=true ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true sync=false max-buffers=1"
    #define OUTPUT_PIPELINE "appsrc ! videoconvert ! osxvideosink sync=false"
    #define CONFIG_PATH "configs/robotConfig.json"

#else   
    #define DEFAULT_INPUT_PIPELINE "libcamerasrc ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true sync=false max-buffers=1"
    #define OUTPUT_PIPELINE "appsrc ! videorate ! videoconvert ! queue ! x264enc tune=zerolatency speed-preset=ultrafast quantizer=25 key-int-max=15 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.32 port=5002 sync=false"
    #define CONFIG_PATH "/home/pi/prog/software/configs/robotConfig.json"
#endif


#include "../common/coms.hpp"
#include "../inc/vision.hpp"
#include "../inc/calibration.hpp"
#include "../inc/navigation.hpp"
#include "../inc/cmdserver.hpp"


int run(int argc, char* argv[]) {
    std::cout << "Starting Vision Pipeline (OpenCV GStreamer)" << std::endl;
    openlog("vision", LOG_PID | LOG_CONS, LOG_USER);

    // --------------- CLI Parsing ----------------- //
    std::string input_pipeline = (argc > 1) ? argv[1] : DEFAULT_INPUT_PIPELINE;
    bool debug_mode = (argc > 2 && std::string(argv[2]) == "--debug");

    // -------------- OpenCV Video Setup ----------------- //
    cv::VideoCapture cap(input_pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        syslog(LOG_ERR, "Failed to open video capture!");
        return -1;
    }

    cv::VideoWriter writer;
    if (!debug_mode) {
        writer.open(OUTPUT_PIPELINE, cv::CAP_GSTREAMER, 0, 30, cv::Size(viz::WIDTH, viz::HEIGHT), true);
        if (!writer.isOpened()) {
            syslog(LOG_ERR, "Failed to open video writer!");
            cap.release();
            return -1;
        }
    }

    // -------------- Runtime Parameters ----------------- //
    std::array<std::atomic<float>, viz::NUM_PARAMS> _params;
    ParameterMap param_map(CONFIG_PATH, viz::NODE_NAME);
    param_map.register_parameter(viz::P_PROG_MODE, _params[viz::P_PROG_MODE], cmd::val_prog_mode);
    param_map.register_parameter(viz::P_LOOK_HRZ_HEIGHT, _params[viz::P_LOOK_HRZ_HEIGHT], cmd::val_hrz_height);
    param_map.register_parameter(viz::P_MAX_VEL, _params[viz::P_MAX_VEL], cmd::val_max_vel);
    param_map.set_value(viz::P_PROG_MODE, viz::M_INIT);
    param_map.set_value(viz::P_LOOK_HRZ_HEIGHT, 1);
    param_map.set_value(viz::P_MAX_VEL, 0.1);

    // ------------- Command Server & Trajectory Threads ----------------- //
    std::thread cmd_thread(cmd::command_server, std::ref(param_map));
    std::thread traj_thread(nav::trajGenServer);

    // -------------- Vision Pipeline Allocations ----------------- //
    cv::Mat homography_matrix;
    cv::Mat undistorted(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat outputFrame(viz::HEIGHT, viz::WIDTH, CV_8UC3);

    // -------------- Camera Calibration ----------------- //
    cv::Mat cameraMatrix, distCoeffs;
    if (!calib::loadCalibration(cameraMatrix, distCoeffs)) {
        syslog(LOG_ERR, "Failed to load camera calibration parameters!");
        cap.release();
        writer.release();
        return -1;
    }

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                               cameraMatrix, cv::Size(viz::WIDTH, viz::HEIGHT), CV_32FC1, map1, map2);

    // -------------- Vision Pipeline Loop ----------------- //
    int loopRate_ms = -1;  
    while (!viz::_exit_trig) {
        // ------------ Read Frame ------------ //
        cv::Mat y_plane;
        if (!cap.read(y_plane)) {
            syslog(LOG_ERR, "Failed to read frame from capture!");
            break;
        }
        auto process_start = std::chrono::high_resolution_clock::now();
        // ------------ Undistort ------------ //
        cv::remap(y_plane, undistorted, map1, map2, cv::INTER_LINEAR);
        // ------------ Processing ------------ //
        viz::img_pipeline(undistorted, outputFrame, homography_matrix, param_map, loopRate_ms);
        auto process_end = std::chrono::high_resolution_clock::now();
        // ------------ Write Frame ------------ //
        if (!outputFrame.empty()) {
            try {
                writer.write(outputFrame);
            } catch (cv::Exception& e) {
                syslog(LOG_ERR, "Failed to write frame to video writer!");
                break;
            }
        }
        // ------------ Loop Performance ------------ //
        auto process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start);
        int loopTime_ms = static_cast<int>(process_duration.count());
        if(loopRate_ms < 0 ){
            if(process_duration.count() > 20) { // high watermark
                syslog(LOG_WARNING, "High watermark %d ms", loopTime_ms);
            }
        }else {
            int sleepTime = debug_mode ? 200 : loopRate_ms;
            std::this_thread::sleep_until(process_start + std::chrono::milliseconds(sleepTime));
        }
        
    }
    // -------------- Cleanup ----------------- //
    cap.release();
    writer.release();
    viz::_exit_trig.store(true);
    cmd_thread.join();
    traj_thread.join();
    closelog();

    return 0;
}

int main(int argc, char* argv[]) {
    #ifdef __APPLE__
    gst_macos_main((GstMainFunc) run, argc, argv, NULL); // Workaround for Gstreamer on MacOS
    #else
    run(argc, argv);
    #endif
    return 0;
}
