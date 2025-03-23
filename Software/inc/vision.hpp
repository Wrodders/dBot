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


float iir_filter(float input, float prev, float alpha) {
    return alpha * input + (1 - alpha) * prev;
}

namespace nav {

    struct Trajectory {
        float w_rate;
        float speed;
    };

    std::queue<Trajectory> _twist_queue;
    std::mutex _twist_mutex;
    std::condition_variable _twist_cv;

    //@brief: Computes velocity references from prominence, confidence and drivability
    void pathFollower(const int peakIdx, const int peakWidth, const int peakProminence) {
        // Normalize the cross-track error to -1 to 1
        float crossTrackError = (peakIdx - 320) / 320.0f;
        bool trackLost = (peakProminence < 50 || peakWidth > 400);
    
        // Trajectory object to hold speed and w_rate
        Trajectory twist;

        if (trackLost) {
            // If track is lost, stop movement and set no rotation
            twist.w_rate = 0.0f;
            twist.speed = 0.0f;
        } else {    
            // Width-based speed adjustment 
            float widthFactor = 1.0f - std::min(peakWidth / 640.0f, 1.0f); // Larger width = slower speed
    
            // Prominence-based speed adjustment
            float prominenceFactor = 1.0f - std::min(peakProminence / 255.0f, 1.0f); // Low prominence = slower speed
    
            // Combined speed factor: both width and prominence affect the speed
            float speedFactor = 0.3f * (widthFactor - prominenceFactor*0.3);
    
            // Limit the maximum speed to 0.2 and ensure it never goes below 0
            twist.speed = std::clamp(speedFactor, 0.0f, 0.3f);
    
            // Adjust rotation rate based on cross-track error
            // deadband to prevent small errors from causing rotation
            crossTrackError = std::abs(crossTrackError) < 0.05f ? 0.0f : crossTrackError;

            twist.w_rate = std::clamp(crossTrackError, -0.6f, 0.6f);
        }
    
        // Debugging output to check the values
        std::cout << "W Rate: " << twist.w_rate << " Speed: " << twist.speed << std::endl;
    
        // Push the trajectory twist to the queue for the robot control
        {
            std::unique_lock<std::mutex> lock(_twist_mutex);
            _twist_queue.push(twist);
            lock.unlock();
            _twist_cv.notify_one();
        }
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
            std::string msg = "<BR" + std::to_string(twist.w_rate) + "\n";
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

enum { M_CALIBRATE = 0, M_PATHFOLLOW,M_PERSUIT, M_SEG_FLOOR, M_VIZ, NUM_MODES };

// -------------- Parameter IDs -------------- //
enum { P_PROG_MODE = 0, P_LOOK_HRZ_HEIGHT, NUM_PARAMS }; 

// -------------- Global Variables -------------- //
std::atomic<bool> _exit_trig(false);

// -------------- Frame Setup -------------- //

const uint16_t WIDTH = 640;
const uint16_t HEIGHT = 480;
const uint32_t FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;

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
    uint16_t floor_y;   // Integer for the floor position (y-coordinate)
};

// Estimate drivability, maintaining proper types for the calculations
WallDetection estimateWallHorizon(const cv::Mat& y_plane) {
    cv::Mat integral_img;
    cv::integral(y_plane, integral_img, CV_32S);
    uint16_t strip_height = 5;
    uint16_t rows = y_plane.rows;
    uint16_t cols = y_plane.cols;
    std::vector<uint16_t> midpoints;
    for (uint16_t y = strip_height; y < rows - strip_height; y++) {
        int sum_above = integral_img.at<int>(y, cols - 1) - integral_img.at<int>(y - strip_height, cols - 1);
        int sum_below = integral_img.at<int>(y + strip_height, cols - 1) - integral_img.at<int>(y, cols - 1);
        if (std::abs(sum_above - sum_below) > 8000) { // large difference in integral sum
            midpoints.push_back(y);
        }
    }
    WallDetection wall_detection = {0};
    if(!midpoints.empty()) {
        std::nth_element(midpoints.begin(), midpoints.begin() + midpoints.size() / 2, midpoints.end());
        float median_horizon = static_cast<float>(midpoints[midpoints.size() / 2]);
        static float current_horizon = median_horizon;
        float horz_lpf = (1 - 0.2f) * current_horizon + 0.2f * median_horizon;
        wall_detection = {
            static_cast<uint16_t>(horz_lpf)
        };
    }

    return wall_detection;
}

//  ROI based on the horizon height
void findRoi(const cv::Mat& y_plane, cv::Rect& roi, ParameterMap& param_map ) {
    float horizon_px;
    (void) param_map.get_value(P_LOOK_HRZ_HEIGHT, horizon_px);
    int horizon = static_cast<int>(horizon_px);
    // Define ROI in original frame
    roi = cv::Rect(0, horizon, WIDTH, HEIGHT - horizon);
}

// Find track estimate using homography transformation
void findTrackEstimate(cv::Mat& roiFrame, const cv::Mat& homography) {
    cv::Mat warped = cv::Mat::zeros(roiFrame.size(), CV_8U);
    //cv::warpPerspective(roiFrame, warped, homography, warped.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    // ------- Edge Detection ------- //
    // Apply Gaussian Blur to reduce noise
    cv::GaussianBlur(roiFrame, roiFrame, cv::Size(21, 5), 0);

    // Apply Sobel operator to detect edges
    cv::Sobel(roiFrame, roiFrame, CV_8U, 1, 0, 3, 2, 0, cv::BORDER_DEFAULT);

    // Apply morphological operations to remove small noise
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3)); // 
    cv::morphologyEx(roiFrame, roiFrame, cv::MORPH_CLOSE, element);
    cv::morphologyEx(roiFrame, roiFrame, cv::MORPH_OPEN, element);



}

// Compute histogram with appropriate types
void computeHistogram(const cv::Mat& edges, cv::Mat& hist) {
    cv::Mat bottom_half = edges(cv::Rect(0, edges.rows / 4, edges.cols, edges.rows *3/ 4));
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);  // Normalize to 0-255 range
    cv::GaussianBlur(hist, hist, cv::Size(21, 1), 0, 0);
}

std::vector<std::tuple<int, int, float>> identifyPeakCandidates(const cv::Mat& hist, float threshold) {
    std::vector<std::tuple<int, int, float>> candidates;

    for (int i = 1; i < hist.cols - 1; ++i) {
        float val = hist.at<float>(0, i);
        
        // Check if the current point is a peak (higher than neighbors)
        if (val > hist.at<float>(0, i - 1) &&
            val > hist.at<float>(0, i + 1) &&
            val > threshold) {

            int leftValley = i, rightValley = i;

            // Find left valley
            while (leftValley > 0 && hist.at<float>(0, leftValley) > threshold)
                leftValley--;

            // Find right valley
            while (rightValley < hist.cols - 1 && hist.at<float>(0, rightValley) > threshold)
                rightValley++;

            // If valleys reach the edges, consider them the maximum possible width
            int width = rightValley - leftValley;

            // Calculate prominence
            // If there are no clear valleys, we use the max histogram value for prominence
            float leftValleyVal = (leftValley > 0) ? hist.at<float>(0, leftValley) : 0.0f;
            float rightValleyVal = (rightValley < hist.cols - 1) ? hist.at<float>(0, rightValley) : 0.0f;

            // Prominence is calculated as the difference between the peak and the maximum of the two valleys
            float prominence = val - std::max(leftValleyVal, rightValleyVal);

            // For cases with a large peak stretching to the edges, prominence can be set to the difference from the peak to the max value in the histogram
            if (leftValley == 0 || rightValley == hist.cols - 1) {
                double minVal, maxVal;
                cv::minMaxLoc(hist, &minVal, &maxVal);
                prominence = val - maxVal;
            }

            candidates.emplace_back(i, width, prominence);
        }
    }

    return candidates;
}

// Compute dynamic prominence for peak identification
std::tuple<float, float> computeDynamicProminence(const std::vector<std::tuple<int, int, float>>& candidates) {
    if (candidates.empty()) return {0, 0};

    std::vector<float> prominences;
    for (const auto& candidate : candidates) {
        prominences.push_back(std::get<2>(candidate));
    }

    float meanProminence = std::accumulate(prominences.begin(), prominences.end(), 0.0f) / prominences.size();
    float maxProminence = *std::max_element(prominences.begin(), prominences.end());
    return {meanProminence, maxProminence - meanProminence};  // Dynamic Range
}

// Find the dominant peak from the candidates
std::tuple<int, int, float> findDominantPeak(
    std::vector<std::tuple<int, int, float>> candidates, float meanProminence, int lastPeakIndex) {

    const float penaltyFactor = 0.8f;
    int bestIndex = -1, bestWidth = 0;
    float bestDynamicRange = -1.0f, bestScore = -std::numeric_limits<float>::max();
    std::vector<int> widths;

    // Loop over all candidates and compute the dynamic range.
    for (const auto& candidate : candidates) {
        int index, width;
        float prominence;
        std::tie(index, width, prominence) = candidate;

        // Calculate the dynamic range: difference between peak's prominence and the mean prominence of candidates.
        float dynamicRange = prominence - meanProminence;

        // Compute score: Dynamic range is the primary factor, penalty for distance from last peak.
        float score = dynamicRange - penaltyFactor * std::abs(index - lastPeakIndex);

        // Track the best score and corresponding peak.
        if (score > bestScore) {
            bestScore = score;
            bestIndex = index;
            bestWidth = width;
            bestDynamicRange = dynamicRange;
        }
    }

    // If we found a valid peak (bestIndex != -1), determine the width based on similar peaks.
    if (bestIndex != -1) {
        widths.push_back(bestWidth);  // Include the width of the dominant peak.

        // Collect widths of peaks with a similar dynamic range to handle uncertainty.
        for (const auto& candidate : candidates) {
            int index, width;
            float prominence;
            std::tie(index, width, prominence) = candidate;

            // Consider peaks with dynamic range close to the best peak's dynamic range.
            if (std::abs(prominence - bestDynamicRange) / std::max(1.0f, bestDynamicRange) < 0.2f) {
                widths.push_back(width);
            }
        }

        // Calculate the final width as the mean of the widths of similar peaks (to represent uncertainty).
        bestWidth = std::accumulate(widths.begin(), widths.end(), 0) / std::max(1, (int)widths.size());
    }

    // Normalize the dynamic range to a value between 0 and 255 to fit within standard range.
    bestDynamicRange = std::min(bestDynamicRange, 255.0f);  // Ensure the dynamic range does not exceed 255.

    return {bestIndex, bestWidth, bestDynamicRange};
}

// Full scan fallback if the best dynamic range is too small
std::tuple<int, int, float> fallbackFullScan(const cv::Mat& hist, float meanProminence, int lastPeakIndex) {
    std::vector<std::tuple<int, int, float>> candidates = identifyPeakCandidates(hist, 0.5f * meanProminence);
    return findDominantPeak(candidates, meanProminence, lastPeakIndex);
}

// Topological peak tracking
std::tuple<uint16_t, uint16_t, uint8_t, cv::Mat> topologicalPeakTrack(const cv::Mat& edges) {
    cv::Mat hist;
    computeHistogram(edges, hist);

    // Identify candidates for peaks from the histogram
    std::vector<std::tuple<int, int, float>> candidates = identifyPeakCandidates(hist, 0.5f * cv::mean(hist)[0]);
    auto [meanProminence, dynamicProminence] = computeDynamicProminence(candidates);

    static int lastPeakIndex = hist.cols / 2; // Initialize the last peak index to the center of the histogram
    auto [bestIndex, bestWidth, bestDynamicRange] = findDominantPeak(candidates, meanProminence, lastPeakIndex);
    // If the dynamic range of the best peak in window is less than a threshold, perform a full scan
    if (bestDynamicRange < 0.8f * dynamicProminence) {
        std::tie(bestIndex, bestWidth, bestDynamicRange) = fallbackFullScan(hist, meanProminence, lastPeakIndex);
    }
    lastPeakIndex = bestIndex;
   
    // ------- Smoothing ------- //
    static float prevFilteredRange = bestDynamicRange;
    prevFilteredRange = iir_filter(bestDynamicRange, prevFilteredRange, 0.3f);
    uint16_t filteredRange = static_cast<uint16_t>(prevFilteredRange);

    static float prevFilteredWidth = bestWidth;
    prevFilteredWidth = iir_filter(bestWidth, prevFilteredWidth, 0.5f);
    bestWidth = static_cast<int>(prevFilteredWidth);

    static float prevFilteredIndex = bestIndex;
    prevFilteredIndex = iir_filter(bestIndex, prevFilteredIndex, 0.5f);
    bestIndex = static_cast<int>(prevFilteredIndex);

    // Return the tuple with best index, width, dynamic range, and the histogram
    return {static_cast<uint16_t>(bestIndex), static_cast<uint16_t>(bestWidth), static_cast<uint8_t>(filteredRange), hist};
}

// Draw topology (visualization) on the frame
void drawTopology(cv::Mat& subFrame, const cv::Mat& hist, int peakIdx, int peakWidth, int peakProminence) {

    subFrame.setTo(0);
    // ------- Visualize Dominant Peak ------- //
    if (peakIdx >= 0) {
        int scaledProminence = static_cast<int>(peakProminence * subFrame.rows / 255.0f);
        cv::Rect peakRect(peakIdx - peakWidth / 2, subFrame.rows - scaledProminence, peakWidth, scaledProminence);
        cv::rectangle(subFrame, peakRect, cv::Scalar(255), -1);  // Visualize as white peak
    }
    // ------- Visualize Histogram ------- //
    for (size_t i = 0; i < hist.cols; i++) {
        int scaledHeight = static_cast<int>(hist.at<float>(0, i) * subFrame.rows / 255.0f);
        cv::line(subFrame, cv::Point(i, subFrame.rows), cv::Point(i, subFrame.rows - scaledHeight),
                    cv::Scalar(128));  // Use grey for histogram lines
    }


}



void tileVisualizer(cv::Mat& y_plane, ParameterMap& param_map, cv::Mat& top_left, cv::Mat& top_right, cv::Mat& bottom_left, cv::Mat& bottom_right) {
    float horizon_px; // UGH NEED to get multiple types 
    (void) param_map.get_value(P_LOOK_HRZ_HEIGHT, horizon_px);
    int horizon = static_cast<int>(horizon_px);
    // ------- Visualization Setup ------- //
    cv::Mat topLeftDst= y_plane(cv::Rect(0, 0, WIDTH / 2, HEIGHT / 2)); // Grab the offsets for each quadrant
    cv::Mat topRightDst = y_plane(cv::Rect(WIDTH / 2, 0, WIDTH / 2, HEIGHT / 2));
    cv::Mat bottomLeftDst = y_plane(cv::Rect(0, HEIGHT / 2, WIDTH / 2, HEIGHT / 2));
    cv::Mat bottomRightDst = y_plane(cv::Rect(WIDTH / 2, HEIGHT / 2, WIDTH / 2, HEIGHT / 2));

    cv::resize(top_left, topLeftDst(cv::Rect(0, horizon / 2, top_left.cols, top_left.rows - (horizon / 2))),
                cv::Size(top_left.cols, top_left.rows - (horizon / 2))); 

    top_right.setTo(0);
    cv::resize(top_right, topRightDst(cv::Rect(0, horizon / 2, top_right.cols, top_right.rows - (horizon / 2))), 
                cv::Size(topRightDst.cols, topRightDst.rows - (horizon / 2)));

    bottom_left.setTo(0);
    cv::resize(bottom_left, bottomLeftDst, cv::Size(bottomLeftDst.cols, bottomLeftDst.rows));

    bottom_right.setTo(0);
    cv::resize(bottom_right, bottomRightDst, cv::Size(bottomRightDst.cols, bottomRightDst.rows));

}

void displayStats(cv::Mat& subFrame, float peakProminence, int peakWidth, float drivability) {
    cv::putText(subFrame, "Dominant Prominence: " + std::to_string(peakProminence),
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(subFrame, "Dominant Width: " + std::to_string(peakWidth),
                cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(subFrame, "Drivability: " + std::to_string(drivability),
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
