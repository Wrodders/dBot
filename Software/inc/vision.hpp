#ifndef VISION_HPP
#define VISION_HPP
#include "../common/coms.hpp"
#include "../common/common.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <istream>
#include <atomic>
#include <thread>
#include <mutex>
#include <deque>
#include <opencv2/opencv.hpp>



namespace nav{
    //@brief: Generates trajectory based on snr of Histogram
    //@param hist: The histogram of the bottom half of the frame
    void algo1(cv::Mat& hist){



        
    }

}

namespace viz {
const char* NODE_NAME = "VISION";
const char* INPUT_PIPE = "///tmp/video_in";
const char* OUTPUT_PIPE =  "///tmp/video_out";

enum {M_CALIBRATE = 0, M_INIT, M_PRE, M_POST, M_RUN, NUM_MODES};
enum{P_CANNY = 0, P_SOBEL, P_LAPLACIAN, NUM_EDGE};

// -------------- Parameter IDs -------------- //
enum{P_PROG_MODE = 0, P_HRZ_HEIGHT, P_TRFM_PAD, P_EDGE, P_NWIN, P_WINPX, 
    P_WIN_MAX, P_CAN_THRESH, P_SOBEL_THRESH, P_LAPLACIAN_THRESH, NUM_PARAMS}; 


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

//@brief: Reads a frame from the input pipe.
bool read_frame(FILE* in_pipe, std::vector<uint8_t>& buffer) {
    size_t bytes_read = fread(buffer.data(), 1, buffer.size(), in_pipe);
    return (bytes_read == FRAME_SIZE);
}
//@brief: Writes a frame to the output pipe.
bool write_frame(FILE* out_pipe, const std::array<uint8_t, FRAME_SIZE>& buffer) {
    size_t bytes_written = fwrite(buffer.data(), 1, buffer.size(), out_pipe);
    return (bytes_written == FRAME_SIZE);
}

//@Brief: Applies an Overlay Mask to an original roi frame to eliminate false edges
//@param frame: The original frame to apply the mask to
//@param dst_pts: The original points of the ROI in the frame
//@param offset: The offset to apply to the ROI mask [pixels]
void apply_internal_mask_roi(cv::Mat& frame, const std::vector<cv::Point2f>& dst_pts, int offset) {
    // Compute centroid of the transformed points
    cv::Point2f centroid(0, 0); 
    for (const auto& pt : dst_pts) {centroid += pt;}
    centroid *= (1.0 / dst_pts.size()); // Normalize the mean
    std::vector<cv::Point> offset_pts(4);  // Offset each point inward along its vector to the centroid
    for (const auto& pt : dst_pts) {   
        cv::Point2f direction = centroid - pt;
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        cv::Point2f unit_direction = direction * (1.0 / length); // Normalize
        cv::Point2f new_pt = pt + unit_direction * offset; // Move point inward by `offset` pixels
        offset_pts.emplace_back(new_pt);
    }
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8U); 
    std::vector<std::vector<cv::Point>> contours = {offset_pts};
    cv::fillPoly(mask, contours, cv::Scalar(255)); // Fill the polygon with white
    frame.setTo(0, mask == 0); // And the mask to the frame 
}



int pipeline(cv::Mat& y_plane, const cv::Mat& homography_matrix, ParameterMap& param_map) {
    //----- Noise Variance Equalization --- //
    cv::equalizeHist(y_plane, y_plane);
    // -------- Floor Segmentation -------- //
    float tf_pad_px; 
    (void) param_map.get_value(P_TRFM_PAD, tf_pad_px);
    float horizon_px;
    (void) param_map.get_value(P_HRZ_HEIGHT, horizon_px);
    cv::Mat roi = y_plane(cv::Rect(0, (HEIGHT - horizon_px), WIDTH, horizon_px));
    // ------- Perspective Transformation ------- //
    cv::warpPerspective(roi, roi, homography_matrix, 
        cv::Size(WIDTH, HEIGHT), cv::INTER_LINEAR, 
        cv::BORDER_CONSTANT, cv::WARP_FILL_OUTLIERS);
    // ------- Edge Detection ------- //
    float edge_mode;
    (void) param_map.get_value(P_EDGE, edge_mode);
    switch((int)edge_mode) {
        case 0:
            cv::Canny(roi, roi, 50, 150, 3);
            break;
        case 1:
            cv::Sobel(roi, roi, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_TRANSPARENT);
            break;
        case 2:
            cv::Laplacian(roi, roi, CV_8U, 3, 1, 0, cv::BORDER_TRANSPARENT);
            break;
        default:
            cv::Canny(roi, roi, 50, 150, 3);
            break;
    }
    // ------- HISTOGRAM ------- //
    cv::Mat bottom_half = roi(cv::Rect(0, HEIGHT / 2, WIDTH, HEIGHT / 2)); // zero-copy
    cv::Mat hist(1, WIDTH, CV_32S); // warm start search with histogram
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32S);

    //---Algorithm 1 ---//
    nav::algo1(hist); // generate trajectory based on histogram
    
    // ------- Sliding Window Search ------- //
    // Take the left and right peaks to warm start the sliding window search
    int midpoint = hist.cols / 2;
    cv::Point left_peak, right_peak; // (x, y) coordinates of the left and right peaks
    cv::minMaxLoc(hist(cv::Rect(0, 0, midpoint, 1)), nullptr, nullptr, nullptr, &left_peak);
    cv::minMaxLoc(hist(cv::Rect(midpoint, 0, midpoint, 1)), nullptr, nullptr, nullptr, &right_peak);

    // ------- Visualization ------- //
    cv::resize(roi,roi, cv::Size(WIDTH, HEIGHT / 2)); // build output frame
    bottom_half.setTo(0);
    for(int i =0; i < bottom_half.cols; i++){
        cv::line(bottom_half, cv::Point(i, HEIGHT / 2), cv::Point(i, HEIGHT / 2 - hist.at<int>(i) / 100), cv::Scalar(255), 1);
    }
    y_plane(cv::Rect(0, HEIGHT / 2, WIDTH, HEIGHT / 2)) = bottom_half;
    y_plane(cv::Rect(0, 0, WIDTH, HEIGHT / 2)) = roi; 

    return 0;
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
//@description: Listens for commands on the VISION topic over IPC;
void command_server(ParameterMap& param_map) {
    fmt::print("[VISION] Starting Command Server\n");
    // --------- Communication Setup --------------------- //
    Protocol _proto = {'<', '\n', ':', 'A'};
    CommandMsg recv_cmd_msg; // Working command variable
    zmq::context_t context(1);
    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, "VISION");
    cmd_subsock.connect("tcp://localhost:5556");  // Connect to Remote Command Server
    cmd_subsock.connect("ipc:///tmp/botcmds");    // Connect to Local Command Server
    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.connect("tcp://localhost:5555"); 
    
    fmt::print("[VISION] Command Server Started\n");
    while(_exit_trig == false){
        coms_receive_asciicmd(cmd_subsock, recv_cmd_msg);
        if (recv_cmd_msg.data == "EXIT") {
            fmt::print("[VISION] Received Exit Command\n");
            break;
        }
        coms_handle_cmd(recv_cmd_msg, msg_pubsock, param_map, _proto, "VISION");
        if(DEBUG_MODE){fmt::print("[VISION] Received Command: {} {} {}\n", recv_cmd_msg.topic, recv_cmd_msg.cmdID, recv_cmd_msg.data);}
    }
    // Cleanup
    _exit_trig.store(true);
    return;
}

static inline bool val_hrz_height(float val) {
    return (val > 0 && val < viz::HEIGHT);
}

static inline bool val_trfm_pad(float val) {
    return (val > 0 && val < viz::WIDTH);
}

static inline bool val_edge(float val) {
    return (val >= 0 && val < NUM_EDGE-1);
}

static inline bool val_prog_mode(float val) {
    return (val >= 0 && val < NUM_MODES);

} // namespace viz
}
#endif // VISION_HPP