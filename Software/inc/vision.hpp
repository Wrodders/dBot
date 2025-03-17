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

#include "../common/coms.hpp"

namespace nav{
    struct Trajectory {
        float angle;
        float speed;
    };
    std::queue<Trajectory> _twist_queue;
    std::mutex _twist_mutex;
    std::condition_variable _twist_cv;

    //@brief: Computes velocity references from prominence, confidence and drivability
    float algo1(const cv::Mat& hist, float* prominence, float* confidence, float* drivability, float* max_speed ) {
        cv::Point lineStartEst;
        cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &lineStartEst);

        float crossTrackError =hist.cols / 2 - lineStartEst.x;
        crossTrackError /= hist.cols / 2;
        crossTrackError = std::clamp(crossTrackError, -0.6f, 0.6f);

        Trajectory twist = {crossTrackError, 0.5};
        // Send Twist Command
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
            {
                std::unique_lock<std::mutex> lock(_twist_mutex);
                _twist_cv.wait(lock, []{return !_twist_queue.empty();});
                Trajectory twist = _twist_queue.front();
                _twist_queue.pop();
                lock.unlock();
                // Build command messages
                std::string msg = "<BR" + std::to_string(twist.angle)+ "\n";
                traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
                traj_pubsock.send(zmq::message_t(msg.c_str(), msg.size()), zmq::send_flags::none);
                std::string msg2 = "<BM" + std::to_string(twist.speed)+ "\n";
                traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
                traj_pubsock.send(zmq::message_t(msg2.c_str(), msg2.size()), zmq::send_flags::none);
            }

        }   
    }
}

namespace viz {
const char* NODE_NAME = "VISION";

enum {M_CALIBRATE = 0, M_RUN, M_SEG_FLOOR, M_VIZ, NUM_MODES};

// -------------- Parameter IDs -------------- //
enum{P_PROG_MODE = 0, P_LOOK_HRZ_HEIGHT, NUM_PARAMS}; 

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
    cv::Point2f((viz::WIDTH / 2 - 90), viz::HEIGHT - 360),
    cv::Point2f((viz::WIDTH / 2 + 90), viz::HEIGHT - 360),
    cv::Point2f(0, 0), 
    cv::Point2f(viz::WIDTH, 0)
};

//@brief: Estimates the drivability of the frame
//@returns: Drivability percentage decimal, higher is better
//@description: Finds the horizon line of the floor and wall. 
float estimateDrivability(const cv::Mat& y_plane) {
    std::vector<int> midpoints;
    float median_horizon;
    
    cv::Mat integral_img; // running sum of the image horizontally
    cv::integral(y_plane, integral_img, CV_32S);

    int strip_height = 5;  // Smaller increases sensitivity 

    int rows = y_plane.rows;
    int cols = y_plane.cols;

    // Scan through horizontal lines
    for (int y = strip_height; y < rows - strip_height; y++) {
        int sum_above = integral_img.at<int>(y, cols - 1) - integral_img.at<int>(y - strip_height, cols - 1);
        int sum_below = integral_img.at<int>(y + strip_height, cols - 1) - integral_img.at<int>(y, cols - 1);

        if (std::abs(sum_above - sum_below) > 5000) {  // large difference -> edge
            midpoints.push_back(y);
        }
    }

    // Find the median horizon line
    std::nth_element(midpoints.begin(), midpoints.begin() + midpoints.size() / 2, midpoints.end());
    median_horizon = (float)midpoints[midpoints.size() / 2];
    static float current_horizon = median_horizon; // inital value
    float horz_lpf = (1-0.2)  * current_horizon + 0.2 * median_horizon;
    return std::clamp(1 - (horz_lpf / viz::HEIGHT), 0.0f, 1.0f);
}


void pipeline(cv::Mat& y_plane, const cv::Mat& homography_matrix, ParameterMap& param_map) {
    // ------- Drivability Estimation ------- //
    int drivability = estimateDrivability(y_plane);
   
    // -------- Floor Segmentation -------- //
    float horizon_px;
    (void) param_map.get_value(P_LOOK_HRZ_HEIGHT, horizon_px);
    int horizon = static_cast<int>(horizon_px);
    // Extract ROI from horizon refencend the top of the image
    cv::Mat roi = y_plane(cv::Rect(0, horizon, WIDTH, HEIGHT - horizon));

    //-- Apply Homography Transformation --//
    cv::Mat warped = cv::Mat::zeros(roi.size(), CV_8U);
    _dst_pts[0].y = _dst_pts[1].y = viz::HEIGHT - horizon;
    
    cv::warpPerspective(roi,warped,
        homography_matrix, warped.size(), cv::INTER_LINEAR,cv::BORDER_CONSTANT, cv::Scalar(0));

    // ------- Edge Detection ------- //
    cv::Mat edges(roi.size(), CV_8U);
    cv::Sobel(warped,edges , CV_8U, 0, 1, 3,1,0);
    //cv::Canny(warped, edges, 80, 150, 3, false);

    // ------- HISTOGRAM ------- //
    cv::Mat bottom_half = edges(cv::Rect(0, edges.rows / 2, edges.cols, edges.rows / 2));
    cv::Mat hist(1, bottom_half.cols, CV_32S); // Warm start search with histogram
    cv::reduce(edges, hist, 0, cv::REDUCE_SUM, CV_32S); 
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);

    //---Algorithm 1 ---//
    float prominence = 0;
    float confidence = 0;
    //nav::algo1(hist, &prominence, &confidence);
    // ------- Visualization ------- //
    // Top-left: ROI, Top-right: Histogram, Bottom-left: Warped, Bottom-right: Edges
    cv::Mat topLeft = y_plane(cv::Rect(0, 0, WIDTH / 2, HEIGHT / 2));    // Grab the offsets of each sub tile 
    cv::Mat topRight = y_plane(cv::Rect(WIDTH / 2, 0, WIDTH / 2, HEIGHT / 2));
    cv::Mat bottomLeft = y_plane(cv::Rect(0, HEIGHT / 2, WIDTH / 2, HEIGHT / 2));
    cv::Mat bottomRight = y_plane(cv::Rect(WIDTH / 2, HEIGHT / 2, WIDTH / 2, HEIGHT / 2));
    
    cv::resize(roi, topLeft(cv::Rect(0, horizon/2, topLeft.cols, topLeft.rows - (horizon/2))), cv::Size(topLeft.cols, topLeft.rows-(horizon/2)));
    topLeft(cv::Rect(0, 0, topLeft.cols, horizon/2)).setTo(0);
    topRight.setTo(0);
    for (int i = 0, j = 0; i < hist.cols && j < topRight.cols; i += 2, ++j) {
        int hist_value = hist.at<int>(i);
        cv::line(topRight, cv::Point(j, HEIGHT / 2), cv::Point(j, HEIGHT / 2 - hist_value), cv::Scalar(255), 1);
    }
    bottomLeft.setTo(0);
    cv::resize(warped, bottomLeft, cv::Size(bottomLeft.cols, bottomLeft.rows));

    bottomRight.setTo(0);
    cv::resize(edges, bottomRight, cv::Size(bottomRight.cols, bottomRight.rows));
    // Display Stats
    cv::putText(topLeft, "Prominence: " + std::to_string(prominence), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(topLeft, "Confidence: " + std::to_string(confidence), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(topLeft, "Drivability: " + std::to_string(drivability), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
}

//@brief: Command Server
//@description: Listens for commands on the VISION topic over TCP and IPC; Publishes cmd rets to the VISION topic o tcp
void command_server(ParameterMap& param_map) {
    syslog(LOG_INFO, "Starting Command Server");
    // --------- Communication Setup --------------------- //
    Protocol _proto = {'<', '\n', ':', 'A'};
    CommandMsg recv_cmd_msg; // Working command variable
    zmq::context_t context(1);

    // Setup subscriber socket
    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, "VISION");
    cmd_subsock.connect("ipc:///tmp/botcmds");    // Connect to Local Command Server
    syslog(LOG_INFO, "Subscribed Cmd Server to ipc:///tmp/botcmds");

    // Setup publisher socket
    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.bind("ipc:///tmp/botmsgs"); // Bind to  Message Server
    syslog(LOG_INFO, "Bound Msg Server to ipc:///tmp/botmsgs");
    while (!_exit_trig) {
        coms_receive_asciicmd(cmd_subsock, recv_cmd_msg);
        coms_handle_cmd(recv_cmd_msg, msg_pubsock, param_map, _proto, "VISION");
    }
    // --------- Cleanup --------------------- //
    syslog(LOG_INFO, "Exiting Command Server");
    context.close();
    _exit_trig.store(true);
}

// -------------- Parameter Validation -------------- //
static inline bool val_hrz_height(float val) {return (val > 0 && val < viz::HEIGHT);}
static inline bool val_trfm_pad(float val) {return (val > 0 && val < viz::WIDTH);}
static inline bool val_prog_mode(float val) {return (val >= 0 && val < NUM_MODES);} 

}
#endif // VISION_HPP