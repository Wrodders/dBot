#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <queue>
#include <condition_variable>   
#include <mutex>
#include <algorithm>
#include <atomic>
#include <array>
#include <vector>
#include <tuple>
#include <zmq.hpp>
#include <syslog.h>
#include <algorithm> 
#include <numeric>
#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include "../common/coms.hpp"

namespace nav {

    struct Trajectory {
        float angle;
        float speed;
    };

    std::queue<Trajectory> _twist_queue;
    std::mutex _twist_mutex;
    std::condition_variable _twist_cv;

    //@brief: Computes velocity references from prominence, confidence and drivability
    float algo1(const cv::Mat& hist, float* prominence, float* confidence, float* drivability, float* max_speed) {
        cv::Point lineStartEst;
        cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &lineStartEst);

        float crossTrackError = hist.cols / 2 - lineStartEst.x;
        crossTrackError /= hist.cols / 2;
        crossTrackError = std::clamp(crossTrackError, -0.6f, 0.6f);

        if (prominence)  *prominence = 0; // placeholder
        if (confidence)  *confidence = 0;
        if (drivability) *drivability = 0;
        if (max_speed)   *max_speed = 0.5;

        Trajectory twist = { crossTrackError, 0.5f };
        {
            std::unique_lock<std::mutex> lock(_twist_mutex);
            _twist_queue.push(twist);
            lock.unlock();
            _twist_cv.notify_one();
        }
        return crossTrackError;
    }

    //@brief: Trajectory Generation Server
    //@description: Publishes twist trajectory commands 
    void trajGenServer(){
        zmq::context_t context(1);
        zmq::socket_t traj_pubsock(context, zmq::socket_type::pub);
        traj_pubsock.set(zmq::sockopt::linger, 0);
        traj_pubsock.bind("ipc:///tmp/vizcmds");
        while(true){
            std::unique_lock<std::mutex> lock(_twist_mutex);
            _twist_cv.wait(lock, []{return !_twist_queue.empty();});
            Trajectory twist = _twist_queue.front();
            _twist_queue.pop();
            lock.unlock();
            // Build command messages
            std::string msg = "<BR" + std::to_string(twist.angle) + "\n";
            traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
            traj_pubsock.send(zmq::message_t(msg.c_str(), msg.size()), zmq::send_flags::none);
            std::string msg2 = "<BM" + std::to_string(twist.speed) + "\n";
            traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
            traj_pubsock.send(zmq::message_t(msg2.c_str(), msg2.size()), zmq::send_flags::none);
        }   
    }
}

namespace viz {

const char* NODE_NAME = "VISION";

enum { M_CALIBRATE = 0, M_RUN, M_SEG_FLOOR, M_VIZ, NUM_MODES };

// -------------- Parameter IDs -------------- //
enum { P_PROG_MODE = 0, P_LOOK_HRZ_HEIGHT, NUM_PARAMS }; 

// -------------- Global Variables -------------- //
std::atomic<bool> _exit_trig(false);

// -------------- Frame Setup -------------- //
const int WIDTH = 640;
const int HEIGHT = 480;
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;

const std::vector<cv::Point2f> _src_pts = {
    cv::Point2f(0, HEIGHT), cv::Point2f(WIDTH, HEIGHT),
    cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
};    

std::array<cv::Point2f, 4> _dst_pts = { // homography transform destination points
    cv::Point2f(WIDTH / 2 - 90, HEIGHT - 360),
    cv::Point2f(WIDTH / 2 + 90, HEIGHT - 360),
    cv::Point2f(0, 0), 
    cv::Point2f(WIDTH, 0)
};

struct WallDetection {
    float drivability;
    int floor_y;
};

//@brief: Estimates the drivability of the frame
//@returns: Drivability percentage decimal, higher is better
//@description: Finds the horizon line of the floor and wall. 
WallDetection estimateDrivability(const cv::Mat& y_plane) {
    cv::Mat integral_img;
    cv::integral(y_plane, integral_img, CV_32S);
    int strip_height = 5;
    int rows = y_plane.rows;
    int cols = y_plane.cols;
    std::vector<int> midpoints;
    for (int y = strip_height; y < rows - strip_height; y++) {
        int sum_above = integral_img.at<int>(y, cols - 1) - integral_img.at<int>(y - strip_height, cols - 1);
        int sum_below = integral_img.at<int>(y + strip_height, cols - 1) - integral_img.at<int>(y, cols - 1);
        if (std::abs(sum_above - sum_below) > 5000) {
            midpoints.push_back(y);
        }
    }
    std::nth_element(midpoints.begin(), midpoints.begin() + midpoints.size() / 2, midpoints.end());
    float median_horizon = static_cast<float>(midpoints[midpoints.size() / 2]);
    static float current_horizon = median_horizon;
    float horz_lpf = (1 - 0.2f) * current_horizon + 0.2f * median_horizon;
    WallDetection wall_detection = {
        std::clamp(1 - (horz_lpf / HEIGHT), 0.0f, 1.0f),
        static_cast<int>(horz_lpf)
    };
    return wall_detection;
}


// Function to find the dominant peak with history tracking
std::tuple<int, float, int> findDominantPeakWidth(const cv::Mat& hist) {
    constexpr float SMOOTHING_FACTOR = 0.3;
    int dominantPeakIndex = -1;
    float dominantProminence = -1.0f;
    int dominantWidth = 0;

    std::vector<float> histVec(hist.begin<float>(), hist.end<float>());
    
    // Compute median and mean
    std::sort(histVec.begin(), histVec.end());
    float medianVal = histVec[histVec.size() / 2];
    float meanVal = std::accumulate(histVec.begin(), histVec.end(), 0.0f) / histVec.size();
    
    // Adaptive thresholding
    float adaptiveThreshold = 0.5f * (medianVal + meanVal);

    // Find local maxima
    for (int i = 1; i < hist.cols - 1; ++i) {
        float currentVal = hist.at<float>(0, i);
        
        // Detect peak
        if (currentVal > hist.at<float>(0, i - 1) && currentVal > hist.at<float>(0, i + 1) && currentVal > adaptiveThreshold) {
            // Find boundaries
            int leftBoundary = i, rightBoundary = i;
            while (leftBoundary > 0 && hist.at<float>(0, leftBoundary) > adaptiveThreshold) leftBoundary--;
            while (rightBoundary < hist.cols - 1 && hist.at<float>(0, rightBoundary) > adaptiveThreshold) rightBoundary++;

            // Compute width & prominence
            int width = rightBoundary - leftBoundary;
            float leftValley = hist.at<float>(0, leftBoundary);
            float rightValley = hist.at<float>(0, rightBoundary);
            float prominence = currentVal - std::max(leftValley, rightValley);

            // Select the dominant peak
            if (prominence > dominantProminence) {
                dominantProminence = prominence;
                dominantPeakIndex = i;
                dominantWidth = width;
            }
        }
    }

    // Temporal Smoothing: If a previous peak exists, apply EMA smoothing
    static int lastPeakIndex = 0.5*hist.cols;
    static float lastPeakProminence = 0;
    static int lastPeakWidth = 0;
    dominantPeakIndex = static_cast<int>(SMOOTHING_FACTOR * dominantPeakIndex + (1 - SMOOTHING_FACTOR) * lastPeakIndex);
    dominantWidth = static_cast<int>(SMOOTHING_FACTOR * dominantWidth + (1 - SMOOTHING_FACTOR) * lastPeakWidth);
    dominantProminence = SMOOTHING_FACTOR * dominantProminence + (1 - SMOOTHING_FACTOR) * lastPeakProminence;
    

    // Update history
    lastPeakIndex = dominantPeakIndex;
    lastPeakWidth = dominantWidth;
    lastPeakProminence = dominantProminence;

    return std::make_tuple(dominantPeakIndex, dominantProminence, dominantWidth);
}

//----------------------------------------------------------------------
// Pipeline Function
// Processes the frame to estimate drivability, perform homography,
// detect edges, build histogram, and then use the enhanced peak detection
// to select a single dominant peak.
//----------------------------------------------------------------------
void pipeline(cv::Mat& y_plane, const cv::Mat& homography_matrix, ParameterMap& param_map) {
    // ------- Drivability Estimation ------- //
    WallDetection wall = estimateDrivability(y_plane);
   
    // -------- Floor Segmentation -------- //
    float horizon_px;
    (void) param_map.get_value(P_LOOK_HRZ_HEIGHT, horizon_px);
    int horizon = static_cast<int>(horizon_px);
    cv::Mat roi = y_plane(cv::Rect(0, horizon, WIDTH, HEIGHT - horizon));
    //-- Apply Homography Transformation --//
    cv::Mat warped = cv::Mat::zeros(roi.size(), CV_8U);
    _dst_pts[0].y = _dst_pts[1].y = HEIGHT - horizon;
    cv::warpPerspective(roi, warped, homography_matrix, warped.size(), cv::INTER_AREA, cv::BORDER_REPLICATE);
    warped(cv::Rect(0, warped.rows / 2, warped.cols, warped.rows / 2)).setTo(0);
    // ------- Edge Detection ------- //
    cv::Mat edges(roi.size(), CV_8U);
    cv::GaussianBlur(warped, warped, cv::Size(5, 5), 0);
    cv::Sobel(warped, edges, CV_8U, 1, 0, 3, 2, 0, cv::BORDER_DEFAULT);    
    // ------- HISTOGRAM ------- //
    cv::Mat bottom_half = edges(cv::Rect(0, edges.rows / 2, edges.cols, edges.rows / 2));
    cv::Mat hist(1, bottom_half.cols, CV_32F);
    cv::reduce(edges, hist, 0, cv::REDUCE_SUM, CV_32F); 
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
    cv::GaussianBlur(hist, hist, cv::Size(21, 1), 0, 0);
    // ------- Topological Peak Tracker ------- //
    int peakIdx;
    float peakProminence;
    int peakWidth;
    std::tie(peakIdx, peakProminence, peakWidth) = findDominantPeakWidth(hist);
    // ------- Visualization Setup ------- //
    cv::Mat topLeft = y_plane(cv::Rect(0, 0, WIDTH / 2, HEIGHT / 2)); // Grab the offsets for each quadrant
    cv::Mat topRight = y_plane(cv::Rect(WIDTH / 2, 0, WIDTH / 2, HEIGHT / 2));
    cv::Mat bottomLeft = y_plane(cv::Rect(0, HEIGHT / 2, WIDTH / 2, HEIGHT / 2));
    cv::Mat bottomRight = y_plane(cv::Rect(WIDTH / 2, HEIGHT / 2, WIDTH / 2, HEIGHT / 2));
    
    cv::resize(roi, topLeft(cv::Rect(0, horizon / 2, topLeft.cols, topLeft.rows - (horizon / 2))),
               cv::Size(topLeft.cols, topLeft.rows - (horizon / 2))); 
    topLeft(cv::Rect(0, 0, topLeft.cols, horizon / 2)).setTo(0);
    topRight.setTo(0);
    // ------- Visualize Dominant Peak ------- //
    if (peakIdx >= 0) {
        cv::Rect peakRect(peakIdx/2 - peakWidth / 2, topRight.rows - 255, peakWidth, 255);
        cv::rectangle(topRight, peakRect, cv::Scalar(255), -1);
    }
    // Draw the histogram (white lines)
    for (int i = 0, j = 0; i < hist.cols && j < topRight.cols; i += 2, ++j) {
        int hist_value = static_cast<int>(hist.at<float>(0, i));
        cv::line(topRight, cv::Point(j, topRight.rows), cv::Point(j, topRight.rows - hist_value), cv::Scalar(128), 1);
    }    
    bottomLeft.setTo(0);
    cv::resize(warped, bottomLeft, cv::Size(bottomLeft.cols, bottomLeft.rows));
    bottomRight.setTo(0);
    cv::resize(edges, bottomRight, cv::Size(bottomRight.cols, bottomRight.rows));
    // ------- Display Stats ------- //
    cv::putText(topLeft, "Dominant Prominence: " + std::to_string(peakProminence),
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(topLeft, "Dominant Width: " + std::to_string(peakWidth),
                cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(topLeft, "Drivability: " + std::to_string(wall.drivability),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
}

//@brief: Command Server
//@description: Listens for commands on the VISION topic over TCP and IPC; publishes command responses to the VISION topic over TCP
void command_server(ParameterMap& param_map) {
    syslog(LOG_INFO, "Starting Command Server");
    Protocol _proto = { '<', '\n', ':', 'A' };
    CommandMsg recv_cmd_msg;
    zmq::context_t context(1);

    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, "VISION");
    cmd_subsock.connect("ipc:///tmp/botcmds");
    syslog(LOG_INFO, "Subscribed Cmd Server to ipc:///tmp/botcmds");

    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.bind("ipc:///tmp/botmsgs");
    syslog(LOG_INFO, "Bound Msg Server to ipc:///tmp/botmsgs");

    while (!_exit_trig) {
        coms_receive_asciicmd(cmd_subsock, recv_cmd_msg);
        coms_handle_cmd(recv_cmd_msg, msg_pubsock, param_map, _proto, "VISION");
    }
    syslog(LOG_INFO, "Exiting Command Server");
    context.close();
    _exit_trig.store(true);
}

// -------------- Parameter Validation -------------- //
static inline bool val_hrz_height(float val) { return (val > 0 && val < HEIGHT); }
static inline bool val_trfm_pad(float val)   { return (val > 0 && val < WIDTH); }
static inline bool val_prog_mode(float val)  { return (val >= 0 && val < NUM_MODES); }

} // namespace viz

#endif // VISION_HPP
