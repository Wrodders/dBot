/************************************************************************************
 *  @file    vision.cpp
 *  @brief   Monocular Vision Trajectory Generations
 *  @date    2025-01-05
 *  @version 0.01
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
 * Target Frame Rate: 25 Hz
 * Output frame for visualization
 * 
 *         640px           
 * |---------------------|
 * | color| Birds| Hist  | 
 * |------| -----|-------| 480px
 * | ROI  | Edge | window|
 * ----------------------|
 ************************************************************************************/
#include "../inc/vision.hpp"
#include <gst/gst.h>
#include <atomic>

#ifdef __APPLE__
#include <TargetConditionals.h>
    #define DEFAULT_INPUT_PIPELINE "avfvideosrc device-index=0 do-timestamp=true ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink"
    #define OUTPUT_PIPELINE "appsrc ! videoconvert ! osxvideosink sync=false"
    #define CONFIG_PATH "configs/robotConfig.json"

#else   
    #define DEFAULT_INPUT_PIPELINE "libcamerasrc ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink"
    #define OUTPUT_PIPELINE "appsrc ! videorate ! videoconvert ! queue ! x264enc tune=zerolatency speed-preset=ultrafast quantizer=25 key-int-max=15 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.32 port=5002 sync=false"
    #define CONFIG_PATH "/home/pi/prog/software/configs/robotConfig.json"
#endif

#define NODE_NAME "VISION"


int run(int argc, char* argv[]) { 
    std::cout << ("Starting Vision Pipeline") << std::endl;
    openlog("vision", LOG_PID | LOG_CONS, LOG_USER);
    // --------------- CLI Parsing ----------------- //
    if (argc < 1) {
        syslog(LOG_ERR, "Usage: %s <input_pipeline>\n", argv[0]);
        return 1;
    }
    std::string input_pipeline = (argc > 1) ? argv[1] : DEFAULT_INPUT_PIPELINE;   
    if (input_pipeline.empty()) {
        syslog(LOG_ERR, "Error: Invalid Input Pipeline\n");
        return 1;
    }
    // -------------- Video IO Setup  ----------------- //
    cv::VideoCapture cap(input_pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        syslog(LOG_ERR, "Failed to open input pipeline!");
        return -1;
    }

    cv::VideoWriter writer(OUTPUT_PIPELINE, cv::CAP_GSTREAMER, 
                           0, 30, cv::Size(viz::WIDTH, viz::HEIGHT), false); // grayscale output
    if (!writer.isOpened()) {
        syslog(LOG_ERR, "Failed to open output pipeline!");
        return -1;
    }
    // -------------- Runtime Parameters ----------------- //
    std::array<std::atomic<float>, viz::NUM_PARAMS> params;
    ParameterMap param_map(CONFIG_PATH, NODE_NAME);
    param_map.register_parameter(viz::P_PROG_MODE, params[viz::P_PROG_MODE], viz::val_prog_mode);
    param_map.register_parameter(viz::P_HRZ_HEIGHT, params[viz::P_HRZ_HEIGHT], viz::val_hrz_height);
    param_map.register_parameter(viz::P_TRFM_PAD, params[viz::P_TRFM_PAD], viz::val_trfm_pad);
    param_map.register_parameter(viz::P_EDGE, params[viz::P_EDGE], viz::val_edge);
    // initialize the parameters
    params[viz::P_PROG_MODE].store(viz::M_CALIBRATE);
    params[viz::P_HRZ_HEIGHT].store(viz::HEIGHT / 2);
    params[viz::P_TRFM_PAD].store(200);
    params[viz::P_EDGE].store(viz::P_CANNY);


    std::array<cv::Point2f, 4> _dst_pts = { // homography transform destination points
        cv::Point2f((viz::WIDTH / 2 - 200), viz::HEIGHT),
        cv::Point2f((viz::WIDTH / 2 + 200), viz::HEIGHT),
        cv::Point2f(0, 0), 
        cv::Point2f(viz::WIDTH, 0)
    };
    
    // ------------- Command Server Coms Thread ----------------- //
    std::thread cmd_thread(viz::command_server, std::ref(param_map));
    // ------------- Tracjectory Publisher Thread ----------------- //
    std::thread traj_thread(nav::trajGen);
    // -------------- Vision Pipeline Allocations ----------------- //
    cv::Mat y_plane(viz::HEIGHT, viz::WIDTH, CV_8UC1); // Single channel for Y plane
    cv::Mat homography_matrix = cv::findHomography(viz::_src_pts, _dst_pts);

    // -------------- Vision Pipeline Loop ----------------- //
    float progmode;
    while (!viz::_exit_trig) {
        // ------------ Read Frame ------------ //
       
        if (!cap.read(y_plane)) {  // Read the full frame
            syslog(LOG_ERR, "Failed to read frame!");
            viz::_exit_trig.store(true); // force exit
            break;
        }
        // ------------ Frame Processing ------------ //
        (void) param_map.get_value(viz::P_PROG_MODE, progmode);
        switch ((int)progmode) {
            case viz::M_CALIBRATE:
                syslog(LOG_INFO, "Calibrating Camera");
                // Run Camera Intrinsic Calibration
                // -- Update Transformation Matrix -- //

                homography_matrix = cv::findHomography(viz::_src_pts, _dst_pts);
                syslog(LOG_INFO, "Calibration Complete");
                params[viz::P_PROG_MODE].store(viz::M_RUN);
                break;
            case viz::M_RUN:
                viz::pipeline(y_plane, homography_matrix, param_map);
                break;
            default:
                break;
        }
        // ------------ Serialize Output Frame ------------ //
        try {
            writer.write(y_plane); // Write the Y plane (grayscale) to output pipeline
        } catch (cv::Exception& e) {
            syslog(LOG_ERR, "Failed to write frame! Sink may be closed.");
            break;
        }
    }
    
    // -------------- Cleanup ----------------- //
    cap.release();
    writer.release();

    viz::_exit_trig.store(true); // force exit
    cmd_thread.join(); 

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