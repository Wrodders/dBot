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

std::array<cv::Point2f, 4> _dst_pts = { // homography transform destination points
    cv::Point2f((viz::WIDTH / 2 - 90), viz::HEIGHT - 360),
    cv::Point2f((viz::WIDTH / 2 + 90), viz::HEIGHT - 360),
    cv::Point2f(0, 0), 
    cv::Point2f(viz::WIDTH, 0)
};


void handleInitMode(ParameterMap& param_map);
void handleCalibBEVMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map, int& loopRate_ms);
void handlePathFollowMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map, viz::PathEstimator& pathEstimator);
void handleSegFloorMode(const cv::Mat& undistorted, cv::Mat& outputFrame);
void handleBirdMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map);
void handleBangBangMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map);
void handleCalibLensMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map);

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
    param_map.set_value(viz::P_KP, 1);
    
    // ------------- Command Server & Trajectory Threads ----------------- //
    std::thread cmd_thread(cmd::command_server, std::ref(param_map));
    std::thread traj_thread(nav::trajGenServer);
    // -------------- Vision Pipeline Allocations ----------------- //
    cv::Mat homography_matrix;
    cv::Mat y_plane(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat undistorted(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat outputFrame(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    // --------------- Navigation Pipeline ----------------- //
    viz::PathEstimator pathEstimator;
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

    if (!calib::loadHomography(homography_matrix)) {
        syslog(LOG_ERR, "Failed to load homography matrix!");
        syslog(LOG_ERR, "Will calibrate the camera");
        param_map.set_value(viz::P_PROG_MODE, viz::M_CALIB_BEV);
    }else {
       syslog(LOG_INFO, "Using loaded homography matrix");
       param_map.set_value(viz::P_PROG_MODE, viz::M_BANG_BANG);
    }

    const std::vector<cv::Point2f> _src_pts = {
        cv::Point2f(0, viz::HEIGHT), cv::Point2f(viz::WIDTH, viz::HEIGHT),
        cv::Point2f(0, 0), cv::Point2f(viz::WIDTH, 0)
    };   
    cv::Mat H_BB = cv::findHomography(_src_pts, _dst_pts);

    // -------------- Performance Metrics ----------------- //
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
        float progmode;
        param_map.get_value(viz::P_PROG_MODE, progmode);
        switch ((int)progmode) {
            case viz::M_INIT:
                handleInitMode(param_map);
                break;
            case viz::M_CALIB_BEV:
                handleCalibBEVMode(undistorted, outputFrame, homography_matrix, param_map, loopRate_ms);
                break;
            case viz::M_BANG_BANG:
                handleBangBangMode(y_plane, outputFrame, H_BB, param_map);
                break;
            case viz::M_PATHFOLLOW:
                handlePathFollowMode(y_plane, outputFrame, homography_matrix, param_map, pathEstimator);
                break;
            case viz::M_SEG_FLOOR:
                handleSegFloorMode(undistorted, outputFrame);
                break;
            case viz::M_BIRD:
                handleBirdMode(undistorted, outputFrame, homography_matrix, param_map);
                break;
            case viz::M_VIZ:
                undistorted.copyTo(outputFrame);
                break;
            default:
                syslog(LOG_ERR, "Invalid Program Mode");
                param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
                break;
        }
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
            if (currentLoopTime > 38) {metrics.hwm_count++;}
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


// ------------------ Main  ----------------- //
int main(int argc, char* argv[]) {
    #ifdef __APPLE__
    gst_macos_main((GstMainFunc) run, argc, argv, NULL); // Workaround for Gstreamer on MacOS
    #else
    run(argc, argv);
    #endif
    return 0;
}


// ------------------ PROGRAM MODES ----------------- //
void handleInitMode(ParameterMap& param_map) {
    syslog(LOG_INFO, "Initializing Vision Pipeline");
    param_map.set_value(viz::P_PROG_MODE, viz::M_PATHFOLLOW);
}

void handleCalibBEVMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map, int& loopRate_ms) {
    static auto start_time = std::chrono::steady_clock::now();
    static bool timeout_active = false;

    if (!timeout_active) {
        start_time = std::chrono::steady_clock::now();
        timeout_active = true;
    }

    float horz_height;
    param_map.get_value(viz::P_LOOK_HRZ_HEIGHT, horz_height);
    std::array<cv::Point2f, 4> imagePts = {};
    int ret = calib::calibrateInversePerspectiveMap(undistorted, homography_matrix, imagePts, horz_height);
    undistorted.copyTo(outputFrame);

    if (ret == 1) {
        syslog(LOG_INFO, "Calibration Complete");
        param_map.set_value(viz::P_PROG_MODE, viz::M_INIT);
        loopRate_ms = -1;
        timeout_active = false;
    } else if (ret == -1) {
        syslog(LOG_ERR, "Calibration Failed");
        param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
        loopRate_ms = -1;
        timeout_active = false;
    } else {
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(60)) {
            syslog(LOG_ERR, "Calibration Timeout");
            param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
            loopRate_ms = -1;
            timeout_active = false;
        } else {
            cv::putText(outputFrame, "Chessboard not found", cv::Point(30, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(64), 2);
            loopRate_ms = 500;
        }
    }
}

void handleSegFloorMode(const cv::Mat& undistorted, cv::Mat& outputFrame) {
    viz::WallDetection walls = viz::estimateWallHorizon(undistorted);
    undistorted.copyTo(outputFrame);
    cv::line(outputFrame, cv::Point(0, walls.floor_y), cv::Point(viz::WIDTH, walls.floor_y), cv::Scalar(64), 10);
}

void handleBirdMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map) {
    if (homography_matrix.empty()) {
        syslog(LOG_ERR, "Homography matrix is empty");
        param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
        return;
    }

    viz::WallDetection walls = viz::estimateWallHorizon(undistorted);
    cv::Mat floor_frame;
    viz::extractFloorPlane(undistorted, floor_frame);
    cv::Mat bev;
    cv::warpPerspective(floor_frame, bev, homography_matrix, outputFrame.size(),
                        cv::WARP_INVERSE_MAP | cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, cv::Scalar(164));
}


//@brief: Path Following by cross track error bang bang control
void handleBangBangMode(const cv::Mat& inputFrame, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map) {
    // Extract ROI from horizon refencend the top of the image
    float horz_height;
    param_map.get_value(viz::P_LOOK_HRZ_HEIGHT, horz_height);
    cv::Mat roi = inputFrame(cv::Rect(0, horz_height, viz::WIDTH, viz::HEIGHT - horz_height));
    // -- Warp Perspective --
    _dst_pts[0].y = _dst_pts[1].y = viz::HEIGHT - horz_height;
    cv::Mat warped_frame;
    cv::warpPerspective(roi,warped_frame,
        homography_matrix, warped_frame.size(), cv::INTER_LINEAR,cv::BORDER_REPLICATE);

    warped_frame(cv::Rect(0, warped_frame.rows / 2, warped_frame.cols, warped_frame.rows / 2)).setTo(0);
    
    // ------- Edge Detection ------- //
    cv::Mat edges(roi.size(), CV_8U);
    cv::Sobel(warped_frame, edges, CV_8U, 1, 0, 3, 1, 0);

    cv::threshold(edges, edges, 40, 255, cv::THRESH_TRUNC);
    // ------- HISTOGRAM ------- //
    cv::Mat hist;
    cv::reduce(edges(cv::Rect(0, edges.rows / 2, edges.cols, edges.rows / 2)), hist, 0, cv::REDUCE_SUM, CV_32F);
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
    cv::GaussianBlur(hist, hist, cv::Size(21, 1), 0);    
    std::vector<float> histVec(hist.cols);
    for (int i = 0; i < hist.cols; ++i) {
        histVec[i] = hist.at<float>(0, i);
    }
    std::vector<float> sortedHist = histVec;
    size_t medianIdx = sortedHist.size() / 2;
    std::nth_element(sortedHist.begin(), sortedHist.begin() + medianIdx, sortedHist.end());
    float median = sortedHist[medianIdx];
    float threshold = median * 1.5f;
    for (auto& val : histVec) {
        if (val < threshold) val = 0;
    }
    cv::Mat histFiltered(1, hist.cols, CV_32F, histVec.data());
    double minVal, maxVal;
    cv::Point lineStartEst;
    cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &lineStartEst);
    nav::bangbang(lineStartEst.x, param_map); 
    // ------- Visualize Histogram ------- //
    cv::resize(edges, outputFrame, outputFrame.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat top_half = outputFrame(cv::Rect(0, 0, edges.cols, edges.rows / 2));
    for (int i = 0; i < hist.cols; ++i) {
        int histValue = static_cast<int>(hist.at<float>(0, i)); // Ensure proper type casting and indexing
        cv::line(top_half, cv::Point(i, top_half.rows), cv::Point(i, top_half.rows - histValue), cv::Scalar(255));
    }

    // ------- Visualize Dominant Peak ------- //
    cv::Rect peakRect(lineStartEst.x - 10, top_half.rows - 20, 20, 20);
    cv::rectangle(top_half, peakRect, cv::Scalar(128), -1);
}

void handlePathFollowMode(const cv::Mat& inputFrame, cv::Mat& outputFrame,cv::Mat& homography_matrix, ParameterMap& param_map, viz::PathEstimator& pathEstimator) {
    // -- Floor Segmentation --
    cv::Mat floor_frame;
    viz::extractFloorPlane(inputFrame, floor_frame);
    cv::Mat bev;
    cv::warpPerspective(floor_frame, bev, homography_matrix, floor_frame.size(),
    cv::WARP_INVERSE_MAP | cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS,
    cv::BORDER_CONSTANT, cv::Scalar(0));
    // -- Track Detection --
    cv::Mat trackFrame;
    viz::detectTrackEdges(bev, trackFrame, homography_matrix);
    // -- Path Tracking --
    //pathEstimator.findPathWaypoints(trackFrame);

    // -- Define the robot's visual position --
    // The frame is 480 pixels tall (30 cm), so scaling yields ~16 px/cm.
    // Given that the robot's position is 25 cm below the bottom, the offset is ~25*16=400 pixels.
    // However, per specification we use (320, 480+300) = (320, 780).
    cv::Point2f robotPos(480 + 300, 320);

    // -- Visualization --
    // Resize the visualizations into the output frame.
    cv::resize(trackFrame, outputFrame, outputFrame.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat topVizFrame = outputFrame(cv::Rect(0, 0, outputFrame.cols, outputFrame.rows / 5));
    cv::Mat midVizFrame = outputFrame(cv::Rect(0, outputFrame.rows * 2/5, outputFrame.cols, outputFrame.rows / 5));
    cv::Mat lowVizFrame = outputFrame(cv::Rect(0, outputFrame.rows * 4/5, outputFrame.cols, outputFrame.rows / 5));

}
