/************************************************************************************
 *  @file    vision.cpp
 *  @brief   Monocular Vision Trajectory Generations
 *  @date    2025-01-05
 *  @version 0.01
 * 
 * Single-threaded Event Driven IO loop
 * Reads frames in YUV from a pipe.
 * For each captured frame generates a continuous trajecotry as a set of linear and angular velociy commands
 * Implements a pub-rpc client over ZMQ, Listens for commands on VIDEO topic over IPC; these modify runtime parameters
 * Performs interactive calibration of the camera using OpenCV and saves the calibration parameters to a file
 * Processes the frames in birds eye view format usign a series of selectable image processing techniques for comparison
 * Selects output frame to pipe as output; this is used to stream the processed video to a web interface
 *                          
 * --------------------------------------------------------------------------------------| 
 * ----> |cmd  | VISION   | ----> |---------|
 * ----> |   stdio        | ----> |         |<---- | pipe | libcamera-vid
 * ----> | msg | TWSB/IMU | ----> | Vision  | 
 * <---- | cmd | TWSB     | <---- |         |----> | ///tmp/video_out | ffmpeg | RTMP | Web Browser 
 * <---- | msg | VISION   | <---- |---------|
 
 ************************************************************************************/


// Pi Camera V3 Parameters




#include "../common/coms.hpp"


#define NODE_NAME "VISION"
#define LIBCAMERA_PIPE "libcamera-vid -t 0 --camera 0 --nopreview --autofocus-mode manual --codec yuv420 --width 640 --height 360 --inline --listen -o -"
#define OUTPUT_PIPE "///tmp/video_out"

// Type Definitions



// ---------- Constants -------------- //
const int WIDTH = 640;
const int HEIGHT = 360;
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;
const int THRESHOLD = 10;

const cv::Vec3b redYUV = cv::Vec3b(81, 90, 240);
const cv::Vec3b greenYUV = cv::Vec3b(145, 54, 34);
const cv::Vec3b blueYUV = cv::Vec3b(41, 240, 110);
const cv::Vec3b yellowYUV = cv::Vec3b(210, 16, 146);

const float grad_dx = 1; // Sobel filter parameters
const float grad_dy = 0;
const float grad_ksize = 3;
const float grad_scale = 1;
const float grad_delta = 0;





// -------- Runtime Parameters ------------ //
float horizon_height = 340;
float transfromPadding = 100;
int nwindows = 9;                     // Number of sliding windows
int margin = 50;                      // Width of the window +/- margin
int minpix = 50;                      // Minimum number of pixels to recenter window
int window_height = HEIGHT / nwindows; // Height of the windowss
float nextOutput = 0;
float nextMode = 0;


// Reads a frame from the input pipe.
bool readFrame(FILE* in_pipe, std::vector<uint8_t>& buffer) {
    size_t bytes_read = fread(buffer.data(), 1, FRAME_SIZE, in_pipe);
    return (bytes_read == FRAME_SIZE);
}

// Writes a frame to the output pipe.
bool writeFrame(FILE* out_pipe, const std::vector<uint8_t>& buffer) {
    size_t bytes_written = fwrite(buffer.data(), 1, FRAME_SIZE, out_pipe);
    return (bytes_written == FRAME_SIZE);
}


//@Brief: Applies an Overlay Mask to an original roi maked frame to eliminate false edges
//@param frame: The original frame to apply the mask to
//@param dst_pts: The original points of the ROI in the frame
//@param offset: The offset to apply to the ROI mask [pixels]
void apply_internal_mask_roi(cv::Mat& frame, const std::vector<cv::Point2f>& dst_pts, int offset) {
    // Compute centroid of the transformed points
    cv::Point2f centroid(0, 0);
    for (const auto& pt : dst_pts) {
        centroid += pt;
    }
    centroid *= (1.0 / dst_pts.size()); // Normalize

    // Offset each point inward along its vector to the centroid
    std::vector<cv::Point> offset_pts;
    for (const auto& pt : dst_pts) {
        cv::Point2f direction = centroid - pt;
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        cv::Point2f unit_direction = direction * (1.0 / length); // Normalize

        // Move point inward by `offset` pixels
        cv::Point2f new_pt = pt + unit_direction * offset;
        offset_pts.emplace_back(new_pt);
    }

    // Create and apply the mask
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8U);
    std::vector<std::vector<cv::Point>> contours = {offset_pts};
    cv::fillPoly(mask, contours, cv::Scalar(255));

    // Remove false boundary edges
    frame.setTo(0, mask == 0);
}

cv::Mat apply_birdsEyeView(const cv::Mat& roi, float transfromPadding, 
                                     std::vector<cv::Point2f>& dst_pts) {
    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(0, HEIGHT), cv::Point2f(WIDTH, HEIGHT),
        cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
    };
    
    dst_pts = {
        cv::Point2f(WIDTH/2 - transfromPadding, HEIGHT),
        cv::Point2f(WIDTH/2 + transfromPadding, HEIGHT),
        cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
    };
    
    cv::Mat homography_matrix = cv::findHomography(src_pts, dst_pts);
    cv::Mat transformed_roi;
    cv::warpPerspective(roi, transformed_roi, homography_matrix, 
                       cv::Size(WIDTH, HEIGHT), cv::INTER_LINEAR, 
                       cv::BORDER_CONSTANT, cv::WARP_FILL_OUTLIERS);
    return transformed_roi;
}


// @brief: Applies a series of image processing techniques to the input frame.
// @param y_plane: The Y-plane of the input YUV420 frame.
// @param output_mode: The output mode to use.
enum ALGO1_OUT {ALGO1_GRAY = 0, ALGO1_ROI, ALGO1_WRAP, ALGO1_GRAD, ALGO1_LINEHIST, ALGO1_NUM_OUT};
cv::Mat filter_algo1(const cv::Mat& y_plane ) {
    // Clamp horizon_height to a valid range.
    horizon_height = std::clamp(horizon_height, 0.0f, float(HEIGHT));
    cv::Mat roi = y_plane(cv::Rect(0, HEIGHT - horizon_height, WIDTH, horizon_height));
    //  Dynamic RPC Apply Birds Eye View Transformation to the ROI - TODO Mode to calibrate
    std::vector<cv::Point2f> src_pts = {cv::Point2f(0, HEIGHT),cv::Point2f(WIDTH, HEIGHT),
                                        cv::Point2f(0, 0),cv::Point2f(WIDTH, 0)
    };
    std::vector<cv::Point2f> dst_pts = {cv::Point2f(WIDTH / 2 - transfromPadding, HEIGHT),
                                        cv::Point2f(WIDTH / 2 + transfromPadding, HEIGHT),
                                        cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
    };

    cv::Mat bev_frame = apply_birdsEyeView(roi, transfromPadding, dst_pts);

    // Apply Sobel filter to the transformed ROI.
    cv::Mat edge_frame;
    cv::Sobel(bev_frame, edge_frame, CV_8U,grad_dx, grad_dy, grad_ksize, grad_scale, grad_delta, cv::BORDER_DEFAULT);
    apply_internal_mask_roi(edge_frame, dst_pts, 10); // Apply internal mask to eliminate false edges from the ROI


    // Sliding Window Warm Start for Line Detection with Histogram
    
    // Compute histogram of the sobel image to help warm start the window search
    // Histogram along the x axis for eahc collums intensity value 
    cv::Mat hist; // 1D Array of the sum of the intensity values of the sobel image for each column 
    // take historgrab of bottom half of the image to get edges close to the car
    cv::Mat bottom_half = edge_frame(cv::Rect(0, HEIGHT / 2, WIDTH, HEIGHT / 2));
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);
    // Normalize the histogram
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


    int left_current = left_peak.x;
    int right_current = right_peak.x;
    // Create Windows using corners
    std::vector<cv::Rect> left_windows;
    for (int i = 0; i < nwindows; i++) {
        int win_y_low = HEIGHT - (i + 1) * window_height;
        int win_y_high = HEIGHT - i * window_height;
        int win_xleft_low = left_current - margin;
        int win_xleft_high = left_current + margin;
        left_windows.emplace_back(cv::Rect(win_xleft_low, win_y_low, win_xleft_high - win_xleft_low, win_y_high - win_y_low));
    }
    std::vector<cv::Rect> right_windows;
    for (int i = 0; i < nwindows; i++) {
        int win_y_low = HEIGHT - (i + 1) * window_height;
        int win_y_high = HEIGHT - i * window_height;
        int win_xright_low = right_current - margin;
        int win_xright_high = right_current + margin;
        right_windows.emplace_back(cv::Rect(win_xright_low, win_y_low, win_xright_high - win_xright_low, win_y_high - win_y_low));
    }


    // Sliding window earhc for lane liens
    // Attempts to find edges inside a window. If enough are found, the window is recentered to the average position of the edges.
    // the window is slid from bottom to top of the image, and the positions of the center of the window are recorded in a vector 
    // 
    // 





    // for the left and right peak 
    cv::Mat output;
    enum ALGO1_OUT output_mode = static_cast<ALGO1_OUT>(std::clamp(int(nextOutput), 0, int(ALGO1_NUM_OUT)));
    switch (output_mode) {
        case ALGO1_GRAY:
            output = y_plane;
            break;
        case ALGO1_ROI:
            output = roi;
            break;
        case ALGO1_WRAP:
            output = bev_frame;
            break;
        case ALGO1_GRAD:
            output = edge_frame;
            break;
        case ALGO1_LINEHIST:
            // For example, you could add histogram visualization here.
            output = hist_image;
            break;
        default:
            output = y_plane;
            break;
    }

    // Resize the output to the full frame size.
    cv::Mat resized_output;
    cv::resize(output, resized_output, cv::Size(WIDTH, HEIGHT));
    return resized_output;
}



enum MODE {CALLIBRATE_CHESS, RUN, NUM_MODE}; // Program modes
int main(int argc, char* argv[]) {
    // -------------- Program Mode FSM ----------------- //
   enum MODE program_mode = RUN;
    // --------- Communication Setup --------------------- //
    Protocol proto = {'<', '\n', ':', 'A'};
    Command cmd; // Working command variable
    std::string cmd_msg; // Command message string
    std::string cmdret_msg; // Command return message string
    TelemetryMsg telem; // Working telemetry variable
    NodeConfigManager config("robotConfig.json", NODE_NAME);
    config.register_parameter(0,&horizon_height);
    config.register_parameter(1,&transfromPadding);
    config.register_parameter(2,&nextOutput);
    //  -------------- IO Setup  ----------------- //
    // Command to capture video using rpicam-vid with YUV420p output
    const char* libcamera_cmd = LIBCAMERA_PIPE;
    FILE* in_pipe = popen(libcamera_cmd, "r");
    if (!in_pipe) {
        std::cerr << "Failed to open pipe to libcamera-vid!" << std::endl;
        return -1;
    }
    // open output pipe as write only file 
    FILE* out_pipe = fopen(OUTPUT_PIPE, "w");
    if (!out_pipe) {
        std::cerr << "Failed to open output pipe!" << std::endl;
        return -1;
    }
    // Make console input non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); 

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
    std::vector<uint8_t> yuv_buffer(FRAME_SIZE); // Input YUV420 frame buffer
    // Horizontal Histogram of Intensity of the edges in the frame
    cv::Mat edge_Hist;

    // ------------ Calibration ----------------- //
    cv::Mat camera_matrix, dist_coeffs;
    cv::Mat map1, map2; // Undistortion maps

    // ----------- Console Preprocessing ----------------- //
    fmt::print("[VISIO]Begin Console\n");
    std::cout << std::flush; 
    // -------------- Event Setup  ----------------- //
    zmq::pollitem_t poll_items[] = { // Poll for ZMQ socket events and pipe events
        {nullptr, fileno(in_pipe), ZMQ_POLLIN, 0},
        {nullptr, STDIN_FILENO, ZMQ_POLLIN, 0 }, // Console input
        {static_cast<void*>(cmd_subsock), 0, ZMQ_POLLIN, 0}}; // ZMQ events}; // Pipe events
    // -------------  Event Loop ----------------- //
    while (true) {
        // Poll for ZMQ socket events and pipe events
        int rc = zmq::poll(poll_items, 3, std::chrono::milliseconds(-1));  
        if (rc == -1) {
            fmt::print("[VISION] Error: Polling Error\n");
            break;
        }        
        /* ********* VISION PIPELINE *********
        @brief: Read YUV420 frame from the pipe
        @description: Reads a frame from the input pipe and processes it using the selected algorithm.
        @returns: coeffiencts of polynomial that describe the line these are sampled based on the pure pursuit algorithm
        @note: Outputs frame to video out pipe 
        */
        if (poll_items[0].revents & ZMQ_POLLIN) { // Frame Available
            if (!readFrame(in_pipe, yuv_buffer)) {
                std::cerr << "[VISION] Error: Unable to read a complete frame." << std::endl;
                break;
            }
            // Proccess Raw Buffer into OpenCV Mat YUV420 Frame
            cv::Mat y_plane(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data()); // opperate on the Y plane only



            cv::Mat processed_frame;
            processed_frame = filter_algo1(y_plane);

            // Combine the grayscale processed frame with the U and V planes, grayed out.
            std::memcpy(yuv_buffer.data(), processed_frame.data, WIDTH * HEIGHT);
            uint8_t* u_plane = yuv_buffer.data() + (WIDTH * HEIGHT);
            uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);
            std::fill(u_plane, u_plane + (WIDTH * HEIGHT / 4), 128);
            std::fill(v_plane, v_plane + (WIDTH * HEIGHT / 4), 128);

            // 2. Write the processed frame to the output pipe.


            // 6. Write the processed frame to the output pipe.
            if (!writeFrame(out_pipe, yuv_buffer)) {
                fmt::print("[VISION] Error: Failed to write output bytes (expected {} bytes).\n", FRAME_SIZE);
                break;
            }

        }
        // ------- Command Processing -------- //
        if (poll_items[1].revents & ZMQ_POLLIN) { // Console Input
            handle_cmdconsole(cmd, cmd_pubsock, config, proto);
        }
        if (poll_items[2].revents & ZMQ_POLLIN) { // Command Message Available
            handle_zmqcmd(cmd, cmd_subsock, msg_pubsock, config, proto);
        }

    }

    // Cleanup
    pclose(in_pipe);
    fclose(out_pipe);
    return 0;
}


