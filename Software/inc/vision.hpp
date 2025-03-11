#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <queue>
#include <condition_variable>   
#include <mutex>


#include "../common/coms.hpp"

namespace nav{

    std::queue<float> _angle_queue;
    std::mutex _angle_mutex;
    std::condition_variable _angle_cv;

    float algo1(const cv::Mat& hist){

        cv::Point lineStartEst;
        cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &lineStartEst);

        float crossTrackError =hist.cols / 2 - lineStartEst.x;
        crossTrackError /= hist.cols / 2;
        std::clamp(crossTrackError, -0.2f, 0.2f);

        {
            std::unique_lock<std::mutex> lock(_angle_mutex);
            _angle_queue.push(-crossTrackError);
            lock.unlock();
            _angle_cv.notify_one();
        }
        return crossTrackError;
    }

    void trajGen(){

        zmq::context_t context(1);
        zmq::socket_t traj_pubsock(context, zmq::socket_type::pub);
        traj_pubsock.set(zmq::sockopt::linger, 0);
        traj_pubsock.bind("ipc:///tmp/vizcmds");


        while(true){
           {
                std::unique_lock<std::mutex> lock(_angle_mutex);
                _angle_cv.wait(lock, []{return !_angle_queue.empty();});
                float angle = _angle_queue.front();
                _angle_queue.pop();
                lock.unlock();

            std::string msg = "<BR" + std::to_string(angle)+ "\n";
            traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
            traj_pubsock.send(zmq::message_t(msg.c_str(), msg.size()), zmq::send_flags::none);
        }

        }






        
    }

   

}

namespace viz {
const char* NODE_NAME = "VISION";


enum {M_CALIBRATE = 0, M_RUN, NUM_MODES};
enum{P_CANNY = 0, P_SOBEL, P_LAPLACIAN, NUM_EDGE};

// -------------- Parameter IDs -------------- //
enum{P_PROG_MODE = 0, P_HRZ_HEIGHT, P_TRFM_PAD, P_EDGE, NUM_PARAMS}; 


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



void pipeline(cv::Mat& y_plane, const cv::Mat& homography_matrix, ParameterMap& param_map) {

    //----- Noise Variance Equalization --- //
    //cv::equalizeHist(y_plane, y_plane);
    // -------- Floor Segmentation -------- //
    float horizon_px;
    (void) param_map.get_value(P_HRZ_HEIGHT, horizon_px);
    int horizon = static_cast<int>(horizon_px);
    // Extract ROI from horizon refencend the top of the image
    cv::Mat roi = y_plane(cv::Rect(0, horizon, WIDTH, HEIGHT - horizon));
    // --- Apply Gaussian Blur --- //
   
    
    //-- Apply Homography Transformation --//
    cv::Mat warped = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);
    cv::warpPerspective(
        roi,
        warped,
        homography_matrix, roi.size() ,cv::INTER_LINEAR | cv::WARP_INVERSE_MAP | cv::WARP_FILL_OUTLIERS);

    // ------- Edge Detection ------- //
    cv::Mat edges(roi.size(), CV_8U);
    cv::Sobel(roi, edges, CV_8U, 1, 0, 3, 1, 0);

    // ------- HISTOGRAM ------- //
    cv::Mat bottom_half = edges(cv::Rect(0, edges.rows / 2, edges.cols, edges.rows / 2));
    cv::Mat hist(1, bottom_half.cols, CV_32S); // Warm start search with histogram
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32S); 
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);




    //---Algorithm 1 ---//
    float snr = nav::algo1(hist);

    // ------- Visualization ------- //
    // Top-left: ROI, Top-right: Histogram, Bottom-left: Warped, Bottom-right: Edges

    // Grab the offsets of each sub tile 
    cv::Mat topLeft = y_plane(cv::Rect(0, 0, WIDTH / 2, HEIGHT / 2));
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

    cv::putText(topRight, "SNR: " + std::to_string(snr), cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    

    bottomLeft.setTo(0);
    cv::resize(warped, bottomLeft, cv::Size(bottomLeft.cols, bottomLeft.rows));

    bottomRight.setTo(0);
    cv::resize(edges, bottomRight, cv::Size(bottomRight.cols, bottomRight.rows));
    

}

//@brief: Searches for chessboard, obtains the homography matrix
bool calibrate(cv::Mat& y_plane, cv::Mat& homography_matrix){
    cv::Mat corners(7, 7, CV_32FC2); 
    std::vector<cv::Point2f> obj_pts(7); // create 3d coordinates of the chessboard in the world reference frame
    for(int i = 0; i < 7; i++){
        for(int j = 0; j < 7; j++){
            obj_pts.push_back(cv::Point2f(j * 40, i * 40));
        }
    }

    std::vector<cv::Point2f> img_pts; // 

    bool found =  cv::findChessboardCorners(y_plane, 
        cv::Size(7,7), // 8x8squares -> 7x7 corners
        corners,        
        cv::CALIB_CB_ADAPTIVE_THRESH |  
        cv::CALIB_CB_FILTER_QUADS);   

    if(!found){ return false;}
    cv::cornerSubPix(y_plane, corners, cv::Size(11, 11), cv::Size(-1, -1), 
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    
    return true;
}


//@brief: Command Server for Vision
//@description: Listens for commands on the VISION topic over TCP and IPC; Publsiehs cmd rets to the VISION topic o tcp
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


static inline bool val_hrz_height(float val) {return (val > 0 && val < viz::HEIGHT);}

static inline bool val_trfm_pad(float val) {return (val > 0 && val < viz::WIDTH);}

static inline bool val_edge(float val) {return (val >= 0 && val < NUM_EDGE-1);}

static inline bool val_prog_mode(float val) {return (val >= 0 && val < NUM_MODES);} 

// namespace viz
}
#endif // VISION_HPP