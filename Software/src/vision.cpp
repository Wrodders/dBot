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


#include "../inc/vision.hpp"


void handleInitMode(ParameterMap& param_map);
void handleCalibBEVMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map, int& loopRate_ms);
void handleSegFloorMode(const cv::Mat& undistorted, cv::Mat& outputFrame);
void handleBirdMode(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map);

void handleAlgo1(const cv::Mat& undistorted, cv::Mat& outputFrame, ParameterMap& param_map);
void handleAlgo2(const cv::Mat& undistorted, cv::Mat& outputFrame, ParameterMap& param_map);
void handleAlgo3(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map);


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
        closelog();
        return -1;
    }
    cv::VideoWriter writer;
    writer.open(OUTPUT_PIPELINE, cv::CAP_GSTREAMER, 0, 25, cv::Size(viz::WIDTH, viz::HEIGHT), false);
    if (!writer.isOpened()) {
        syslog(LOG_ERR, "Failed to open video writer!");
        cap.release();
        closelog();
        return -1;
    }
    // -------------- Runtime Parameters ----------------- //
    std::array<std::atomic<float>, viz::NUM_PARAMS> _params;
    ParameterMap param_map(CONFIG_PATH, viz::NODE_NAME);
    param_map.register_parameter(viz::P_PROG_MODE, _params[viz::P_PROG_MODE], cmd::val_prog_mode);
    param_map.register_parameter(viz::P_LOOK_HRZ_HEIGHT, _params[viz::P_LOOK_HRZ_HEIGHT], cmd::val_hrz_height);
    param_map.register_parameter(viz::P_MAX_VEL, _params[viz::P_MAX_VEL], cmd::val_max_vel);
    param_map.register_parameter(viz::P_NAV_EN, _params[viz::P_NAV_EN], cmd::val_nav_en);
    param_map.register_parameter(viz::P_KP, _params[viz::P_KP], cmd::val_gain);
    param_map.register_parameter(viz::P_KI, _params[viz::P_KI], cmd::val_gain);
    param_map.register_parameter(viz::P_KD, _params[viz::P_KD], cmd::val_gain);
    
    param_map.set_value(viz::P_PROG_MODE, viz::M_INIT);
    param_map.set_value(viz::P_LOOK_HRZ_HEIGHT,360);
    param_map.set_value(viz::P_MAX_VEL, 0.1);
    param_map.set_value(viz::P_NAV_EN, 1);
    param_map.set_value(viz::P_KP, 1);
    param_map.set_value(viz::P_KI, 0);
    param_map.set_value(viz::P_KD, 0);
    
    // ------------- Command Server & Trajectory Threads ----------------- //
    std::thread cmd_thread(cmd::command_server, std::ref(param_map));
    std::thread traj_thread(nav::trajGenServer);
    // -------------- Vision Pipeline Allocations ----------------- //
    cv::Mat homography_matrix;
    cv::Mat y_plane(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat undistorted(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    cv::Mat outputFrame(viz::HEIGHT, viz::WIDTH, CV_8UC1);
    // --------------- Navigation Pipeline ----------------- //
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
       param_map.set_value(viz::P_PROG_MODE, viz::M_PATHFOLLOW);
    }

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
                handleAlgo1(y_plane, outputFrame,param_map);
                break;
            case viz::M_PATHFOLLOW:
                handleAlgo2(y_plane, outputFrame, param_map);
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
            int sleepTime = debug_mode ? 200 : loopRate_ms; // Rate Limit for debug mode
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
    int ret = calib::calibrateInversePerspectiveMap(undistorted, homography_matrix, imagePts);
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
    cv::Mat floor_frame;
    viz::extractFloorPlane(undistorted, floor_frame);
    cv::Mat bev;
    cv::warpPerspective(floor_frame, bev, homography_matrix, outputFrame.size(),
                        cv::WARP_INVERSE_MAP | cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, cv::Scalar(164));
}


void handleAlgo1(const cv::Mat& inputFrame, cv::Mat& outputFrame, ParameterMap& param_map) {
    static nav::PIDController pid(1.0f, 0.0, 0.0f);
    // ------- Look Ahead Section------- //
    float horz_height;
    param_map.get_value(viz::P_LOOK_HRZ_HEIGHT, horz_height);
    cv::Mat roi = inputFrame(cv::Rect(0, horz_height, viz::WIDTH, viz::HEIGHT - horz_height));  
    // ------- Track Detection ------- //
    cv::medianBlur(roi, roi, 21);
    cv::Mat edges;
    cv::Sobel(roi, edges, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::Mat section = edges(cv::Rect(0, edges.rows / 2, edges.cols, edges.rows / 2));
    std::array<float, 640> hist_trend;;
    struct pk::LineTrend line_trend = pk::computeLineTrend(section, hist_trend);
    float median = line_trend.median;
    float max = line_trend.max;
    float threshold = 15 * median;
    // ------- Stop and search if track lost ------- //
    static float prev_estidx = 320;
    float speed, est_idx;
    if (max < threshold) { // Track lost
        prev_estidx = iir_lpf(prev_estidx, 320, 0.001f); // Exponential decay to center
        est_idx = prev_estidx; // Apply filtered value
        speed = 0.0f; // throttle speed
    } else {
        est_idx = line_trend.index; // Use the peak index of the histogram
        (void) param_map.get_value(viz::P_MAX_VEL, speed);
    }
    // ------- Control ------- //
    float crossTrackError = nav::computeCrossTrackError(est_idx);
    nav::point_regulator(-crossTrackError, speed, pid, param_map);
    
    // ------- Visualizer ------- //
    cv::resize(edges, outputFrame, outputFrame.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat viz_section = outputFrame(cv::Rect(0, 0, outputFrame.cols, outputFrame.rows / 2));    
    for (std::size_t i = 0; i < hist_trend.size(); ++i) {
        
        int histValue = static_cast<int>(hist_trend[i]);
        cv::line(viz_section, cv::Point(i, viz_section.rows), cv::Point(i, viz_section.rows - histValue), cv::Scalar(255));
    }
    
    cv::Rect peakRect(est_idx - 10, viz_section.rows - 20, 20, 20);
    cv::rectangle(viz_section, peakRect, cv::Scalar(128), -1);
}

void handleAlgo2(const cv::Mat& inputFrame, cv::Mat& outputFrame,ParameterMap& param_map) {
    static nav::PIDController pid(1.0f, 0.0, 0.0f);
    // ------- Adaptive Look Ahead Section ------- //
    viz::WallDetection walls = viz::estimateWallHorizon(inputFrame);
    cv::Mat roi = inputFrame(cv::Rect(0, walls.floor_y, viz::WIDTH, viz::HEIGHT - walls.floor_y));
    cv::medianBlur(roi, roi, 21);
    // ------- Track Detection ------- //
    cv::Mat track_edges;
    cv::Sobel(roi, track_edges, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    bool track_lost = false;
    const int numSections = 5;
    const int sectionHeight = track_edges.rows / numSections;
    std::vector<std::tuple<pk::Peak, int>> waypoints;
    static std::array<std::tuple<pk::Peak, int>, numSections> prev_waypoints = {};
    std::array<cv::Rect, numSections> sectionsRoi = {};
    pk::PeakTracker tracker;
    // --- SearchWhole image for waypoints ---
    for(int i = 0; i < numSections; ++i) {
        int sectionStart = track_edges.rows - (i + 1) * sectionHeight;
        cv::Rect roi = cv::Rect(0, sectionStart, track_edges.cols, sectionHeight);
        sectionsRoi[i] = roi;
        cv::Mat section = track_edges(roi);
        pk::Peak peak = tracker.topologicalPeakTrack(section, std::get<0>(prev_waypoints[i]));
        track_lost = (peak.prominence < 100 || peak.width > 200); // Noise floor is too high
        if(!track_lost ) {
            waypoints.push_back(std::make_tuple(peak, i));
        }
    }
    cv::resize(track_edges, outputFrame, outputFrame.size(), 0, 0, cv::INTER_LINEAR);
    float speed, est_idx;
    pk::Peak& peak = std::get<0>(prev_waypoints[0]);
    if(track_lost){ // Stop and search
        peak.index = iir_lpf(peak.index, 320, 0.1f); // Exponential decay to center;
        est_idx = peak.index;
        speed = 0.0f;
    }else { // Carve into path
        int pointsFound = waypoints.size();
        assert( pointsFound > 0);
        printf("Found %d waypoints\n", pointsFound);
        (void) param_map.get_value(viz::P_MAX_VEL, speed);
        speed*=pointsFound/numSections; // Slow Down when less of the track is detected
        est_idx = std::get<0>(waypoints[0]).index; // use the closest peak found
        // -- Waypoint Visualization --
        for (int i = 0; i < pointsFound; ++i) {
            pk::Peak peak = std::get<0>(waypoints[i]);
            int sectionIdx = std::get<1>(waypoints[i]);
            cv::Rect vis_sectRoi = sectionsRoi[sectionIdx];
            vis_sectRoi.height /=2;
            cv::Mat subMat = outputFrame(vis_sectRoi);
            viz::drawTopology(subMat, peak, cv::Scalar(192));
        }
        // update the previous waypoints
        for (int i = 0; i < numSections; ++i) {
            std::get<0>(prev_waypoints[i]) = std::get<0>(waypoints[i]);
        }
    }
    // -- Control --
    float crossTrackError = nav::computeCrossTrackError(est_idx);
    nav::point_regulator(-crossTrackError, speed, pid, param_map);

    return;
}

void handleAlgo3(const cv::Mat& inputFrame, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map) {
    // -- Project Track to BEV -- 
    cv::Mat floor_frame;
    viz::extractFloorPlane(inputFrame, floor_frame);
    cv::Mat bev;
    cv::warpPerspective(floor_frame, bev, homography_matrix, floor_frame.size(),
    cv::WARP_INVERSE_MAP | cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS,
    cv::BORDER_CONSTANT, cv::Scalar(0));
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
