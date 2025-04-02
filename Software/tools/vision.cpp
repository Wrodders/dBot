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
    #define DEFAULT_INPUT_PIPELINE "libcamerasrc ! video/x-raw,format=I420,width=640,height=480,framerate=25/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true sync=false max-buffers=1"
    #define OUTPUT_PIPELINE "appsrc ! videorate ! videoconvert ! queue ! x264enc tune=zerolatency speed-preset=ultrafast quantizer=25 key-int-max=15 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.32 port=5002 sync=false"
    #define CONFIG_PATH "/home/pi/prog/software/configs/robotConfig.json"
#endif


#include "../common/coms.hpp"
#include "../inc/vision.hpp"
#include "../inc/calibration.hpp"
#include "../inc/navigation.hpp"
#include "../inc/cmdserver.hpp"


struct PerfMetrics {
    long loop_count = 0;
    int hwm_count =0;
    double total_process = 0;
    double total_write = 0;
    double peak_process = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_report;
    
    void reset() {
        loop_count = 0;
        hwm_count = 0;
        total_process = 0;
        peak_process = 0;
        total_write = 0;
        last_report = std::chrono::steady_clock::now();
    }
};

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
        writer.open(OUTPUT_PIPELINE, cv::CAP_GSTREAMER, 0, 25, cv::Size(viz::WIDTH, viz::HEIGHT), false);
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
    param_map.register_parameter(viz::P_NAV_EN, _params[viz::P_NAV_EN], cmd::val_nav_en);
    param_map.register_parameter(viz::P_KP, _params[viz::P_KP], cmd::val_kp);
    
    param_map.set_value(viz::P_PROG_MODE, viz::M_INIT);
    param_map.set_value(viz::P_LOOK_HRZ_HEIGHT,360);
    param_map.set_value(viz::P_MAX_VEL, 0.1);
    param_map.set_value(viz::P_NAV_EN, 1);
    param_map.set_value(viz::P_KP, 15);
    
    // ------------- Command Server & Trajectory Threads ----------------- //
    std::thread cmd_thread(cmd::command_server, std::ref(param_map));
    std::thread traj_thread(nav::trajGenServer);
    // -------------- Vision Pipeline Allocations ----------------- //
    cv::Mat homography_matrix;
    cv::Mat y_plane(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat undistorted(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat outputFrame(viz::HEIGHT, viz::WIDTH, CV_8UC1);
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
                               cameraMatrix, cv::Size(viz::WIDTH, viz::HEIGHT), CV_16SC2, map1, map2);

    // -------------- Performance Metrics ----------------- //
        // In main loop
    PerfMetrics metrics;
    constexpr int REPORT_INTERVAL_MS = 30000;

    // -------------- Vision Pipeline Loop ----------------- //
    int loopRate_ms = -1;  
    while (!viz::_exit_trig) {
        // ------------ Read Frame ------------ //
        if (!cap.read(y_plane)) {
            syslog(LOG_ERR, "Failed to read frame from capture!");
            break;
        }
        auto process_start = std::chrono::steady_clock::now();
        // ------------ Undistort ------------ //
        cv::remap(y_plane, undistorted, map1, map2, cv::INTER_LINEAR);
        // ------------ Processing ------------ //
        viz::img_pipeline(undistorted, outputFrame, homography_matrix, param_map, loopRate_ms);
        auto process_end = std::chrono::steady_clock::now();
        // ------------ Write Frame ------------ //
        if (!outputFrame.empty()) {
            try {
                writer.write(outputFrame);
            } catch (cv::Exception& e) {
                syslog(LOG_ERR, "Failed to write frame to video writer!");
                break;
            }
        }
        auto write_end = std::chrono::steady_clock::now();
        // ------------ Loop Performance ------------ //
        if (loopRate_ms < 0) { // Normal operation no rate limiting
            metrics.loop_count++;
            // Update performance metrics
            double currentLoopTime = std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start).count();
            metrics.total_process += currentLoopTime;
            metrics.total_write += std::chrono::duration_cast<std::chrono::milliseconds>(write_end - process_end).count();
            metrics.peak_process = std::max(metrics.peak_process, currentLoopTime);
            if (currentLoopTime > 30) {
            metrics.hwm_count++;
            }
            // Periodic Performance Report 
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - metrics.last_report).count() > REPORT_INTERVAL_MS) {
            if (metrics.hwm_count > 0) { // Only log if high water mark was reached
                syslog(LOG_WARNING, "Loop Count: %ld, HWM Count: %d, Avg Process Time: %.4f ms, Peak Process Time: %.4f ms, Avg Write Time: %.4f ms",
                metrics.loop_count, metrics.hwm_count,
                (metrics.total_process / metrics.loop_count),
                (metrics.peak_process),
                (metrics.total_write / metrics.loop_count));
            }
            metrics.reset();
            }
        } else {
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
