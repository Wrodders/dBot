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

#include "../common/common.hpp"
#include "../common/coms.hpp"
#include "../inc/calibration.hpp"
#include "../inc/peakTracker.hpp"


namespace viz {

const char* NODE_NAME = "VISION";

// -------------- Program Modes -------------- //
enum { M_INIT = 0, M_CALIB_BEV, M_CALIB_LENS, M_BANG_BANG, M_PATHFOLLOW, M_SEG_FLOOR, M_BIRD, M_VIZ, NUM_MODES };
// -------------- Parameter IDs -------------- //
enum { P_PROG_MODE = 0, P_LOOK_HRZ_HEIGHT, P_MAX_VEL, P_NAV_EN, P_KP, P_KI, P_KD, NUM_PARAMS }; 
// -------------- Global Variables -------------- //
std::atomic<bool> _exit_trig(false);
// -------------- Frame Setup -------------- //
const uint16_t WIDTH = 640;
const uint16_t HEIGHT = 480;

// ************************ Feature Detection ************************ //
struct WallDetection {
    uint16_t floor_y;   // Integer for the floor position (y-coordinate)
    float angle;  // Angle of the wall  wrt camera 
};

struct PathArc {
    float curvature;
    float radius;
    float angle;
    float length;
};

//@brief: Estimate the horizon height of the floor
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
    WallDetection wall_detection = {0, 0.0f};
    if (!midpoints.empty()) {
        std::nth_element(midpoints.begin(), midpoints.begin() + midpoints.size() / 2, midpoints.end());
        float median_horizon = static_cast<float>(midpoints[midpoints.size() / 2]);
        median_horizon = std::clamp(median_horizon, 0.0f, static_cast<float>(y_plane.rows) / 5); // Clamp to valid range
        static float prev_horizon = median_horizon;
        
        float horz_lpf = iir_lpf(median_horizon, prev_horizon, 0.2f);
        prev_horizon = horz_lpf; // Update the previous horizon value for the next iteration

        // Estimate wall angle using linear regression
        std::vector<cv::Point> edge_points;
        for (uint16_t y = 0; y < rows; y++) {
            for (uint16_t x = 0; x < cols; x++) {
                if (y_plane.at<uint8_t>(y, x) > 128) { // Threshold to find edges
                    edge_points.emplace_back(x, y);
                }
            }
        }
        static float current_angle = 0.0f; // Declare static variable outside the block
        if (!edge_points.empty()) {
            cv::Vec4f line;
            cv::fitLine(edge_points, line, cv::DIST_L2, 0, 0.01, 0.01);
            float angle = 0.0f;
            angle = std::atan2(line[1], line[0]) * 180.0f / M_PI; // Calculate angle in radians
            angle = std::clamp(angle, -5.0f, 5.0f); // Limit angle 
            float angle_lpf = iir_lpf(angle, current_angle, 0.1f);  // Low-pass filter the angle
            current_angle = angle_lpf; // Update the static variable
        }

        wall_detection = {
            static_cast<uint16_t>(horz_lpf), current_angle
        };
    }

    return wall_detection;
}

void extractFloorPlane(const cv::Mat& y_plane, cv::Mat& floor_frame) {
    // Estimate the floor horizon and angle
    viz::WallDetection walls = viz::estimateWallHorizon(y_plane);
    int look_hrz_height = static_cast<int>(walls.floor_y);
    const cv::Mat roi_frame = y_plane(cv::Rect(0, look_hrz_height, viz::WIDTH, viz::HEIGHT - look_hrz_height));
    const cv::Mat rotation_matrix = cv::getRotationMatrix2D(
        cv::Point2f(viz::WIDTH / 2, viz::HEIGHT / 2), -walls.angle, 1.0);
    cv::Mat rotated_frame;
    cv::warpAffine(roi_frame, floor_frame, rotation_matrix, roi_frame.size(),
                   cv::INTER_AREA, cv::BORDER_CONSTANT, cv::Scalar(0));
}

void detectTrackEdges(const cv::Mat& frame, cv::Mat& edges) {
    // ------- Edge Detection ------- //
    //cv::GaussianBlur(frame, edges, cv::Size(5, 21), 0);
    cv::Sobel(frame, edges, CV_8U, 1, 0, 3, 2, 0, cv::BORDER_DEFAULT);
    cv::medianBlur(edges, edges, 5);
    //  mask warped img boundaries edges fromm sobel
    std::vector<cv::Point> contour = {
        cv::Point(10, 0),
        cv::Point(frame.cols-10, 0),
        cv::Point(frame.cols/2 +200, frame.rows),
        cv::Point(frame.cols/2 -200, frame.rows)
    };

    cv::line(edges, contour[0], contour[3], cv::Scalar(0), 30);
    cv::line(edges, contour[1], contour[2], cv::Scalar(0), 30); // Mask warped img boundaries
}

class PathEstimator {
public:
    PathEstimator() :
        topTracker(640), midTracker(640), lowTracker(640),
        top_pathTrend{0}, mid_pathTrend{0}, low_pathTrend{0},
        topRoi(cv::Rect(0, 0, 640, 480 / 5)),
        midRoi(cv::Rect(0, 480 * 2 / 5, 640, 480 / 5)),
        lowRoi(cv::Rect(0, 480 * 4 / 5, 640, 480 / 5)) {
        }
    void findPathWaypoints(const cv::Mat& trackFrame){
        // -- Waypoint Detection -
        pkt::Peak topPeak = topTracker.topologicalPeakTrack(trackFrame, topRoi, top_pathTrend);
        pkt::Peak midPeak = midTracker.topologicalPeakTrack(trackFrame, midRoi, mid_pathTrend);
        pkt::Peak lowPeak = lowTracker.topologicalPeakTrack(trackFrame, lowRoi, low_pathTrend);
    }

    void findPathArc(){
        // -- Path Arc Estimation --

    }

    inline const PathArc& getPathArc() const {return pathArc;}

    private:
        pkt::PeakTracker topTracker;
        pkt::PeakTracker midTracker;
        pkt::PeakTracker lowTracker;
        std::array<float, WIDTH> top_pathTrend;
        std::array<float, WIDTH> mid_pathTrend;
        std::array<float, WIDTH> low_pathTrend;
        const cv::Rect topRoi;
        const cv::Rect midRoi;
        const cv::Rect lowRoi;
        PathArc pathArc;
};


// ******************************* Visualization ******************************* //
void drawTopology(cv::Mat& subFrame, const std::array<float, WIDTH>& hist, int peakIdx, int peakWidth, float peakProminence, cv::Scalar histColor, cv::Scalar peakColor) {
    //subFrame.setTo(0);
    // ------- Visualize Dominant Peak ------- //

    // ------- Visualize Histogram ------- //
    for (size_t i = 0; i < hist.size(); ++i) {
        int scaledHeight = static_cast<int>(hist[i] * subFrame.rows / 255.0f);
        cv::line(subFrame, cv::Point(i, subFrame.rows), cv::Point(i, subFrame.rows - scaledHeight), histColor);    }

    if (peakIdx >= 0) {
        int scaledProminence = static_cast<int>(peakProminence * subFrame.rows / 255.0f);
        cv::Rect peakRect(peakIdx - peakWidth / 2, subFrame.rows - scaledProminence, peakWidth, scaledProminence);
        cv::rectangle(subFrame, peakRect, peakColor, -1);
    }
}

void displayStats(cv::Mat& subFrame, float peakProminence, int peakWidth, float drivability) {
    cv::putText(subFrame, "Dominant Prominence: " + std::to_string(peakProminence),
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(subFrame, "Dominant Width: " + std::to_string(peakWidth),
                cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(subFrame, "Drivability: " + std::to_string(drivability),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
}

} // namespace viz


namespace nav {

struct Trajectory {
    float w_rate;
    float speed;
};

std::queue<Trajectory> _twist_queue;
std::mutex _twist_mutex;
std::condition_variable _twist_cv;

//@Brief: Simple Point Tracking Algorithm - Bang Bang
//@Description: Computes the cross-track error based on the histogram of the image
//              Sets angular velocity opposing the error. 
//@Note        Sets linear velocity to max speed parameter
void bangbang(const int peakIdx, const float maxSpeed, ParameterMap& param_map) {
    float crossTrackError = 320 - peakIdx;
    crossTrackError /= 320 / 2; // Normalize error

    // PID state variables
    static float lastError = 0.0f;
    static float integral = 0.0f;
    static auto lastTime = std::chrono::steady_clock::now();
    // Get time delta
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsedTime = now - lastTime;
    float dt = elapsedTime.count();
    lastTime = now;
    float Kp, Ki, Kd;
    (void) param_map.get_value(viz::P_KP, Kp);
    (void) param_map.get_value(viz::P_KI, Ki);
    (void) param_map.get_value(viz::P_KD, Kd);
    // PID parameters

    // PID calculations
    integral += crossTrackError * dt;               // Accumulate integral
    integral = std::clamp(integral, -0.5f, 0.5f); // Limit integral
    float derivative = (crossTrackError - lastError) / dt; // Calculate derivative
    lastError = crossTrackError;

    float controlSignal = (Kp * crossTrackError) + (Ki * integral) + (Kd * derivative);
    controlSignal = std::clamp(controlSignal, -0.6f, 0.6f); // Limit output

    // Construct trajectory command
    struct Trajectory twist = {.w_rate = -controlSignal, .speed = maxSpeed};

    // Push to control queue
    {
        std::unique_lock<std::mutex> lock(_twist_mutex);
        _twist_queue.push(twist);
    }
    _twist_cv.notify_one();
}

//@Brief: Slows down speed based on curve width and confidance, 
void slowOnCurves(const int peakIdx, const int curveWidth, const int curveConfidance, ParameterMap& param_map) {
    // Normalize the cross-track error to -1 to 1
    float crossTrackError = viz::WIDTH / 2 - peakIdx;
    crossTrackError /= viz::WIDTH / 2;
    crossTrackError = std::clamp(crossTrackError, -0.2f, 0.2f);
    bool trackLost = (curveConfidance < 55 || curveWidth > 600);
    // Trajectory object to hold speed and w_rate
    Trajectory twist;
    if (trackLost) {
        // If track is lost, stop movement and set no rotation
        twist.w_rate = 0.0f;
        twist.speed = 0.0f;
    } else {    
        float curveFactor =  curveWidth / 600.0f;  // normalize to 0-1
        float confidenceFactor = curveConfidance / 200.0f;
        float maxSpeed;
        (void) param_map.get_value(viz::P_MAX_VEL, maxSpeed);

        twist.speed = maxSpeed * (1 - curveFactor) * confidenceFactor;
        twist.speed = std::clamp(twist.speed, 0.0f, maxSpeed);
        twist.w_rate = -crossTrackError;      
    }
    // Push the trajectory twist to the queue for the robot control
    {
        std::unique_lock<std::mutex> lock(_twist_mutex);
        _twist_queue.push(twist);
        lock.unlock();
        _twist_cv.notify_one();
    }
}


// @brief: pure pursuit algorithm

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


namespace cmd {
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

    while (!viz::_exit_trig) {
        coms_receive_asciicmd(cmd_subsock, recv_cmd_msg);
        coms_handle_cmd(recv_cmd_msg, msg_pubsock, param_map, _proto, "VISION");
    }
    syslog(LOG_INFO, "Exiting Command Server");
    context.close();
    viz::_exit_trig.store(true);
}

// -------------- Parameter Validation -------------- //
static inline bool val_hrz_height(float val) { return (val > 0 && val < viz::HEIGHT); }
static inline bool val_trfm_pad(float val)   { return (val > 0 && val < viz::WIDTH); }
static inline bool val_prog_mode(float val)  { return (val >= 0 && val < viz::NUM_MODES); }
static inline bool val_max_vel(float val)    { return (val >= 0 && val < 1); }
static inline bool val_nav_en(float val)     { return (static_cast<int>(val) % 2 == 0 || static_cast<int>(val) % 2 == 1); } // test if 0 or 1
static inline bool val_kp(float val)        { return (val >= 0 && val < 100); } // test if 0 or 1

} // namespace cmd

#endif // VISION_HPP
