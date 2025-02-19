/************************************************************************************
 *  @file    vision.cpp
 *  @brief   Monocular Vision Trajectory Generations
 *  @date    2025-01-05
 *  @version 0.01
 * 
 * Gstreamer Pipeline:
 * Reads frames in YUV from a pipe.
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
#include "../common/coms.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <istream>
#include <atomic>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>


#define NODE_NAME "VISION"
#define INPUT_PIPE "///tmp/video_in"
#define OUTPUT_PIPE "///tmp/video_out"

// -------------- Global Variables -------------- //
bool _exit_coms_thread = false;

// ---------- Constants -------------- //
const int WIDTH = 640;
const int HEIGHT = 480;
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;

// Color Constants
const cv::Vec3b redYUV = cv::Vec3b(81, 90, 240);
const cv::Vec3b greenYUV = cv::Vec3b(145, 54, 34);
const cv::Vec3b blueYUV = cv::Vec3b(41, 240, 110);
const cv::Vec3b yellowYUV = cv::Vec3b(210, 16, 146);
// ------ Sobel Filter Parameters ------ //
const float grad_dx = 1;    //
const float grad_dy = 0;    //
const float grad_ksize = 3; //
const float grad_scale = 1; //
const float grad_delta = 0; //
// 
const std::vector<cv::Point2f> src_pts = {
cv::Point2f(0, HEIGHT), cv::Point2f(WIDTH, HEIGHT),
    cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
};    

// -------- Runtime Parameters ------------ //
struct Parameters{
    std::atomic<float> horizon_height_px = 20.0f;
    std::atomic<float> transform_bottom_padding_px = 20.0f;
    std::atomic<float> next_edgemode = 0;
    std::atomic<int> n_swindows = 9;
    std::atomic<int> swin_margin_px = 100;
    std::atomic<int> swin_min_px = 50;
    std::atomic<float> next_prgmode = 0;
};
struct Parameters _params;

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

//@brief: Perspective geometry transformation
cv::Mat apply_birdsEyeView(const cv::Mat& roi, const std::vector<cv::Point2f>& dst_pts) {
    static cv::Mat homography_matrix = cv::findHomography(src_pts, dst_pts);
    cv::Mat transformed_roi;
    cv::warpPerspective(roi, transformed_roi, homography_matrix, 
                       cv::Size(WIDTH, HEIGHT), cv::INTER_LINEAR, 
                       cv::BORDER_CONSTANT, cv::WARP_FILL_OUTLIERS);
    return transformed_roi;
}

//@brief: Command Server for Vision
//@description: Listens for commands on the VISION topic over IPC;
//
void command_server(){
    // --------- Communication Setup --------------------- //
    Protocol _proto = {'<', '\n', ':', 'A'};
    CommandMsg _cmd; // Working command variable
    NodeConfigManager config("configs/robotConfig.json", NODE_NAME);
    ZmqCmdServer cmd_server(NODE_NAME, CMD_SOCKET_ADDRESS, MSG_PUB_ADDRESS);
    config.register_parameter(0,&_params.horizon_height_px);
    config.register_parameter(1,&_params.transform_bottom_padding_px);
    config.register_parameter(2,&_params.next_edgemode);
    
    // ---------------  Event Loop ----------------- //
    zmq::pollitem_t poll_items[] = {
        {static_cast<void*>(cmd_server.cmd_subsock), 0, ZMQ_POLLIN, 0},
        {static_cast<void*>(STDIN_FILENO), 0, ZMQ_POLLIN, 0}    
    };
    while(_exit_coms_thread == false){
        int rc = zmq::poll(poll_items, 2, std::chrono::milliseconds(33));  // 30 Hz
        if (rc == -1) {
            fmt::print("[VISION] Error: Polling Error\n");
            break;
        }   
        // ------------ ZMQ COMMAND EVENT ------------ //
        if (poll_items[0].revents & ZMQ_POLLIN) { // Command Message Available
            if(handle_zmqcmd(_cmd, cmd_server.cmd_subsock, cmd_server.msg_pubsock, cmd_server.cmd_pubsock, config, _proto) < 0){
                _exit_coms_thread = true;
                break;
            }
        }
    }
    // Cleanup
    _exit_coms_thread = true;
    return;
}




int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;
    std::cout << "[VISION] Starting Vision Pipeline" << std::endl;
    //  -------------- Video IO Setup  ----------------- //
    mkfifo(INPUT_PIPE, 0666); 
    mkfifo(OUTPUT_PIPE, 0666); 
    // Command to capture video using rpicam-vid with YUV420p output
    FILE* in_pipe = fopen(INPUT_PIPE, "r");
    if (!in_pipe) {
        std::cerr << "Failed to open pipe to video input!" << std::endl;
        return -1;
    }
   
    FILE* out_pipe = fopen(OUTPUT_PIPE, "w");
    if (!out_pipe) {
        std::cerr << "Failed to open pipe to video output!" << std::endl;
        return -1;
    }
    // ------------- Command Server Coms Thread ----------------- //
    std::thread cmd_thread(command_server);
    // ------------- Pipeline Preprocessing ----------------- //
    static std::array<uint8_t, FRAME_SIZE> yuv_buffer; // serialized frame buffer 
    static cv::Mat composite_y(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data());
    std::fill(yuv_buffer.begin() + WIDTH*HEIGHT, yuv_buffer.end(), 128);

    constexpr std::chrono::milliseconds TARGET_FRAME_TIME(33); // ~30fps
    auto last_frame_time = std::chrono::system_clock::now();
    while (_exit_coms_thread == false) {     
        /* ********* VISION PIPELINE *********
        @brief: Read YUV420 frame from the pipe
        @description: Reads a frame from the input pipe and processes it using the selected algorithm.
        @returns: coeffs of polynomial that describe the line these are sampled based on the pure pursuit algorithm
        @note: Outputs frame to video out pipe 
        */
       auto loop_start = std::chrono::system_clock::now();

        size_t bytes_read = fread(yuv_buffer.data(), 1, yuv_buffer.size(), in_pipe);
        if (bytes_read != FRAME_SIZE) {
            std::cerr << "[VISION] Error: Unable to read a complete frame." << std::endl;
            continue; // skip frame if not complete
        }
        // Process the gray scale Y plane of the YUV420 frame
        cv::Mat y_plane(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data()); // CAST AND CLAMP 
        int tf_pad_px = (int)_params.transform_bottom_padding_px;
        cv::Mat floor_segment_roi = y_plane(cv::Rect(0, (HEIGHT - _params.horizon_height_px), 
                                                        WIDTH, _params.horizon_height_px));
        
        const std::vector<cv::Point2f> dst_pts = {  
            cv::Point2f((WIDTH / 2 - tf_pad_px), HEIGHT),
            cv::Point2f((WIDTH / 2 + tf_pad_px), HEIGHT),
            cv::Point2f(0, 0), 
            cv::Point2f(WIDTH, 0)
        };
        // Apply perspective transformation to the ROI
        cv::Mat wrap_frame = apply_birdsEyeView(floor_segment_roi, dst_pts);
        // Apply Sobel filter to the transformed ROI.
        cv::Mat edge_frame;
        int edge_mode = (int)_params.next_edgemode;
        switch(edge_mode) {
            case 0:
                cv::Canny(wrap_frame, edge_frame, 50, 150, 3);
                break;
            case 1:
                cv::Sobel(wrap_frame, edge_frame, CV_8U, grad_dx, grad_dy, grad_ksize, grad_scale, grad_delta, cv::BORDER_DEFAULT);
                break;
            case 2:
                cv::Laplacian(wrap_frame, edge_frame, CV_8U, grad_ksize, grad_scale, grad_delta, cv::BORDER_DEFAULT);
                break;
            default:
                cv::Canny(wrap_frame, edge_frame, 50, 150, 3);
                break;
        }
        apply_internal_mask_roi(edge_frame, dst_pts, 10); // Apply internal mask to eliminate false edges from the ROI
    
        // Compute histogram of the sobel image for warm start
        cv::Mat hist(edge_frame.size().width, 1, CV_32F); 
        cv::Mat bottom_half = edge_frame(cv::Rect(0, HEIGHT / 2, WIDTH, HEIGHT / 2)); 
        cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);
        cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);  
    
        // Map the histogram to a visualization image for debugging
        cv::Mat hist_image = cv::Mat::zeros(edge_frame.size(), CV_8UC1);
        for (int i = 0; i < hist.cols; i++) {
            cv::line(hist_image, cv::Point(i, HEIGHT), cv::Point(i, HEIGHT - hist.at<float>(0, i)), cv::Scalar(255), 1);
        }
        // Take the left and right peaks to warm start the sliding window search
        int midpoint = hist.cols / 2;
        cv::Point left_peak, right_peak; // (x, y) coordinates of the left and right peaks
        cv::minMaxLoc(hist(cv::Rect(0, 0, midpoint, 1)), nullptr, nullptr, nullptr, &left_peak);
        cv::minMaxLoc(hist(cv::Rect(midpoint, 0, midpoint, 1)), nullptr, nullptr, nullptr, &right_peak);
        int left_current = left_peak.x; // initialize window search start points
        int right_current = right_peak.x;

        const cv::Size targetSize(WIDTH/4, HEIGHT/2);
        cv::Mat composite_y(HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0));
        int tile_width = WIDTH / 4;
        int tile_height = HEIGHT / 2;
        // Resize and tile the Y-plane images
        std::vector<cv::Mat> out_planes = {y_plane, wrap_frame, edge_frame, hist_image};
        for (int i = 0; i < 4; i++) {
            cv::Mat resized_tile;
            cv::resize(out_planes[i], resized_tile, cv::Size(tile_width, tile_height));
            // Determine placement position in the grid
            int row = i / 4;
            int col = i % 4;
            cv::Rect roi(col * tile_width, row * tile_height, tile_width, tile_height);
            resized_tile.copyTo(composite_y(roi));
        }
        // Copy the composite Y-plane into the YUV buffer
        std::memcpy(yuv_buffer.data(), composite_y.data, WIDTH * HEIGHT);
        // Fill the U and V planes with 128 (neutral chroma)
        uint8_t* u_plane = yuv_buffer.data() + (WIDTH * HEIGHT);
        uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);
        std::fill(u_plane, u_plane + (WIDTH * HEIGHT / 4), 128);
        std::fill(v_plane, v_plane + (WIDTH * HEIGHT / 4), 128);
        try {
            write_frame(out_pipe, yuv_buffer);
        } catch (const std::exception& e) {
            fmt::print("[{}] Error: {}\n", NODE_NAME, e.what());
        }
        auto processing_time = std::chrono::system_clock::now() - loop_start;
        auto sleep_time = TARGET_FRAME_TIME - processing_time;
        last_frame_time = loop_start;
    }
    // Cleanup
    fclose(in_pipe);
    fclose(out_pipe);
    _exit_coms_thread = true;
    cmd_thread.join(); 
    fmt::print("[{}] Exiting\n", NODE_NAME);
    return 0;
}


