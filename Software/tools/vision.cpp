/************************************************************************************
 *  @file    vision.cpp
 *  @brief   Monocular Vision Trajectory Generations
 *  @date    2025-01-05
 *  @version 0.01
 * 
 * Single-threaded Event Driven IO loop
 * Reads frames in YUV from a pipe.
 * For each captured frame generates a continuous trajectory as a set of linear and angular velocity commands
 * Implements a pub-rpc client over ZMQ, Listens for commands on VIDEO topic over IPC; these modify runtime parameters
 * Performs interactive calibration of the camera using OpenCV and saves the calibration parameters to a file
 * Processes the frames in birds eye view format usign a series of selectable image processing techniques for comparison
 * Selects output frame to pipe as output; this is used to stream the processed video to a web interface
 *                          
 * ------------------------------------------------|
 * ----> |cmd  | VISION   | ----> |---------|      |-----------------------------------|
 * ----> |   stdio        | ----> |         |<---- | pipe | libcamera-vid or GStreamer |
 * ----> | msg | TWSB/IMU | ----> | Vision  |      |-----------------------------------|
 * <---- | cmd | TWSB     | <---- |         |----> | pipe | ffmpeg or GStreamer        | 
 * <---- | msg | VISION   | <---- |---------|      |-----------------------------------|
 * 
 * 
 * output frame for visualization
 * 
 *              640px           
 * |--------------------------|
 * | yuv  | Birds| Edge | Hist| 
 * |------|      |      |-----| 300px
 * | ROI  | Eye  |      | yuv |
 * ----------------------------
 * 
 * 
 * 
 * 
 ************************************************************************************/
#include "../common/coms.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <istream>
#include <thread>

#define NODE_NAME "VISION"
#define INPUT_PIPE "///tmp/video_in"
#define OUTPUT_PIPE "///tmp/video_out"

// ---------- Constants -------------- //
const int WIDTH = 640;
const int HEIGHT = 480;
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;

const cv::Vec3b redYUV = cv::Vec3b(81, 90, 240);
const cv::Vec3b greenYUV = cv::Vec3b(145, 54, 34);
const cv::Vec3b blueYUV = cv::Vec3b(41, 240, 110);
const cv::Vec3b yellowYUV = cv::Vec3b(210, 16, 146);

const float grad_dx = 1; // Sobel filter parameters
const float grad_dy = 0;
const float grad_ksize = 3;
const float grad_scale = 1;
const float grad_delta = 0;

const std::vector<cv::Point2f> src_pts = {
cv::Point2f(0, HEIGHT), cv::Point2f(WIDTH, HEIGHT),
    cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
};    

// -------- Runtime Parameters ------------ //
float _horizon_height_px = 340; // Relative to the bottom of the frame
float _transform_bottom_padding_px = 100; // Relative to the center of the frame
int _n_swindows = 9;        // Number of sliding windows
int _win_margin_px = 50;    // Width of the window +/- margin
int _swin_minpx = 50;      // Minimum number of pixels to recenter window
float _next_edgemode = 0;
float _next_prgmode = 0;


// Reads a frame from the input pipe.
bool read_frame(FILE* in_pipe, std::vector<uint8_t>& buffer) {
    size_t bytes_read = fread(buffer.data(), 1, buffer.size(), in_pipe);
    return (bytes_read == FRAME_SIZE);
}
// Writes a frame to the output pipe.
bool write_frame(FILE* out_pipe, const std::vector<uint8_t>& buffer) {
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
    std::vector<cv::Point> offset_pts;  // Offset each point inward along its vector to the centroid
    for (const auto& pt : dst_pts) {    // iterate over the 4 points
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
    cv::Mat homography_matrix = cv::findHomography(src_pts, dst_pts);
    cv::Mat transformed_roi;
    cv::warpPerspective(roi, transformed_roi, homography_matrix, 
                       cv::Size(WIDTH, HEIGHT), cv::INTER_LINEAR, 
                       cv::BORDER_CONSTANT, cv::WARP_FILL_OUTLIERS);
    return transformed_roi;
}

// Utility function to letterbox an image into a given tile size.
// It scales the source image preserving its aspect ratio and centers it on a black background.
cv::Mat letterboxImage(const cv::Mat& src, int dest_width, int dest_height) {
    // Create a black tile of the target size.
    cv::Mat tile = cv::Mat::zeros(dest_height, dest_width, src.type());
    // Determine the scaling factor.
    double scale = std::min((double)dest_width / src.cols, (double)dest_height / src.rows);
    int new_w = static_cast<int>(src.cols * scale);
    int new_h = static_cast<int>(src.rows * scale);
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));
    // Center the resized image in the tile.
    int offset_x = (dest_width - new_w) / 2;
    int offset_y = (dest_height - new_h) / 2;
    cv::Rect roi(offset_x, offset_y, new_w, new_h);
    resized.copyTo(tile(roi));
    return tile;
}

std::queue<std::string> cmd_queue;
std::mutex queue_mutex;
std::condition_variable condv;
bool exit_thread = false;

// Function to read console input in a blocking manner
void read_console_input() {
    while (!exit_thread) {
        std::string cmd_input;
        std::getline(std::cin, cmd_input);  // Read line blocking
        if (!cmd_input.empty()) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            cmd_queue.push(cmd_input);  // Push input into queue
            condv.notify_one();  // Notify main thread about new input
        }
    }
}



enum EDGE_ALGOMODE {EDGE_CANNY = 0, EDGE_SOBEL, EDGE_LAPLACE, NUM_EDGEALGO};
enum MODE {CALIBRATE_CHESS, RUN, NUM_MODE}; // Program modes
int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;
    fmt::print("[VISION] Begin Vision\n");
    // -------------- Program Mode FSM ----------------- //
   enum MODE program_mode = RUN;
   enum EDGE_ALGOMODE edge_mode = EDGE_CANNY;
    // --------- Communication Setup --------------------- //
    Protocol _proto = {'<', '\n', ':', 'A'};
    Command _cmd; // Working command variable
    std::string _cmd_msg; // Command message string
    std::string _cmdret_msg; // Command return message string
    TelemetryMsg _telem; // Working telemetry variable
    NodeConfigManager config("configs/robotConfig.json", NODE_NAME);
    config.register_parameter(0,&_horizon_height_px);
    config.register_parameter(1,&_transform_bottom_padding_px);
    config.register_parameter(2,&_next_edgemode);
    //  -------------- Video IO Setup  ----------------- //
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
    // ------------- Interactive Console Setup ----------------- //
    std::thread console_thread(read_console_input);

    // -------------- ZMQ Setup ----------------- //
    zmq::context_t context(1);
    zmq::socket_t cmd_subsock = zmq::socket_t(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.connect(CMD_SOCKET_ADDRESS); // Connect to the command socket proxy
    cmd_subsock.set(zmq::sockopt::subscribe, NODE_NAME);
    // Command Writer
    zmq::socket_t cmd_pubsock = zmq::socket_t(context, zmq::socket_type::pub);
    cmd_pubsock.set(zmq::sockopt::linger, 0);
    cmd_pubsock.connect(CMD_SOCKET_ADDRESS); // Connect to the command socket proxy
    // Bot Msgs
    zmq::socket_t msg_pubsock = zmq::socket_t(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.connect(MSG_PUB_ADDRESS); // Connect to the message socket proxy
    // ------------- Pipeline Preprocessing ----------------- //
    std::vector<uint8_t> yuv_buffer(FRAME_SIZE); // serialized frame buffer 
    // ------------ Calibration ----------------- //
    cv::Mat camera_matrix, dist_coeffs;
    cv::Mat map1, map2; 
    // -------------- Event Setup  ----------------- //
    zmq::pollitem_t poll_items[] = { // Poll for ZMQ socket events and pipe events
        {nullptr, fileno(in_pipe), ZMQ_POLLIN, 0}, // Video Input Pipe events
        {static_cast<void*>(cmd_subsock), 0, ZMQ_POLLIN, 0}}; // ZMQ events};
    // -------------  Event Loop ----------------- //
    fmt::print("[VISIO]Begin Console\n");
    while (true) {
        int rc = zmq::poll(poll_items, 2, std::chrono::milliseconds(33));  // 30 Hz
        if (rc == -1) {
            fmt::print("[VISION] Error: Polling Error\n");
            break;
        }        
        /* ********* VISION PIPELINE *********
        @brief: Read YUV420 frame from the pipe
        @description: Reads a frame from the input pipe and processes it using the selected algorithm.
        @returns: coeffs of polynomial that describe the line these are sampled based on the pure pursuit algorithm
        @note: Outputs frame to video out pipe 
        */
       if (poll_items[0].revents & ZMQ_POLLIN) { // Frame Available
            size_t bytes_read = fread(yuv_buffer.data(), 1, yuv_buffer.size(), in_pipe);
            if (bytes_read != FRAME_SIZE) {
                std::cerr << "[VISION] Error: Unable to read a complete frame." << std::endl;
                continue; // skip frame if not complete
            }
        
            // Process the gray scale Y plane of the YUV420 frame
            cv::Mat y_plane(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data());
            _horizon_height_px = std::clamp(_horizon_height_px, 20.0f, float(HEIGHT));
            cv::Mat floor_segment_roi = y_plane(cv::Rect(0, HEIGHT - _horizon_height_px, WIDTH, _horizon_height_px));
            
            const std::vector<cv::Point2f> dst_pts = {  
                cv::Point2f((WIDTH / 2 - _transform_bottom_padding_px), HEIGHT),
                cv::Point2f((WIDTH / 2 + _transform_bottom_padding_px), HEIGHT),
                cv::Point2f(0, 0), 
                cv::Point2f(WIDTH, 0)
            };
            // Apply perspective transformation to the ROI
            cv::Mat wrap_frame = apply_birdsEyeView(floor_segment_roi, dst_pts);
        
            // Apply Sobel filter to the transformed ROI.
            cv::Mat edge_frame;
            edge_mode = static_cast<EDGE_ALGOMODE>(std::clamp(int(_next_edgemode), 0, int(NUM_EDGEALGO -1)));
            switch(edge_mode) {
                case EDGE_CANNY:
                    cv::Canny(wrap_frame, edge_frame, 50, 150, 3);
                    break;
                case EDGE_SOBEL:
                    cv::Sobel(wrap_frame, edge_frame, CV_8U, grad_dx, grad_dy, grad_ksize, grad_scale, grad_delta, cv::BORDER_DEFAULT);
                    break;
                case EDGE_LAPLACE:
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

            cv::line(composite_y, cv::Point(WIDTH / 4, 0), cv::Point(WIDTH / 4, HEIGHT), blueYUV, 1);
            cv::line(composite_y, cv::Point(WIDTH / 2, 0), cv::Point(WIDTH / 2, HEIGHT), blueYUV, 1);
            cv::line(composite_y, cv::Point(3 * WIDTH / 4, 0), cv::Point(3 * WIDTH / 4, HEIGHT), blueYUV, 1);
            cv::line(composite_y, cv::Point(0, HEIGHT / 2), cv::Point(WIDTH, HEIGHT / 2), blueYUV, 1);
            cv::line(composite_y, cv::Point(0, HEIGHT - _horizon_height_px), cv::Point(WIDTH, HEIGHT - _horizon_height_px), blueYUV, 1);
            cv::line(composite_y, cv::Point(WIDTH / 2 - _transform_bottom_padding_px, HEIGHT), cv::Point(WIDTH / 2 - _transform_bottom_padding_px, 0), blueYUV, 1);
            cv::line(composite_y, cv::Point(WIDTH / 2 + _transform_bottom_padding_px, HEIGHT), cv::Point(WIDTH / 2 + _transform_bottom_padding_px, 0), blueYUV, 1);
            // Copy the composite Y-plane into the YUV buffer
            std::memcpy(yuv_buffer.data(), composite_y.data, WIDTH * HEIGHT);
            
            // Fill the U and V planes with 128 (neutral chroma)
            uint8_t* u_plane = yuv_buffer.data() + (WIDTH * HEIGHT);
            uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);
            std::fill(u_plane, u_plane + (WIDTH * HEIGHT / 4), 128);
            std::fill(v_plane, v_plane + (WIDTH * HEIGHT / 4), 128);

            // draw grid lines in blue using the YUV color space

            // Draw the sliding windows on the composite image



            // Write to output pipe (assuming write_frame is defined elsewhere)
            try {
                write_frame(out_pipe, yuv_buffer);
            } catch (const std::exception& e) {
                std::cerr << "[VISION] Error: Unable to write frame to output pipe\n";
            }
        }
    
        // ------------ ZMQ COMMAND EVENT ------------ //
        if (poll_items[1].revents & ZMQ_POLLIN) { // Command Message Available
            if(handle_zmqcmd(_cmd, cmd_subsock, msg_pubsock, config, _proto) == -1) {
                break;
            }
        }
        // ------------ CONSOLE COMMAND QUEUE HANDLER ------------ //
        while(!cmd_queue.empty()){
            // Process the command queue
            std::lock_guard<std::mutex> lock(queue_mutex);
            std::string cmd_input = cmd_queue.front();
            cmd_queue.pop();
            if (cmd_input == "exit") {
                fmt::print("[VISION] Exiting\n");
                // Cleanup
                fclose(in_pipe);
                fclose(out_pipe);
                exit_thread = true;                
                return 0;
            }
            if(cmd_input.length() < 2) { 
                fmt::print(" >> Invalid Command\n");
                continue;;
            }
            proto_pack_asciicmd(cmd_input, _proto); // Console UX is protocol agnostic
            if (proto_deserialize_cmd(cmd_input, _proto, _cmd)) {
                coms_exec_rpc(_cmd, config, msg_pubsock);
            } else { fmt::print(" >> Invalid Command\n"); continue;}
        }
    }
    // Cleanup
    fclose(in_pipe);
    fclose(out_pipe);
    exit_thread = true;
    return 0;
}


