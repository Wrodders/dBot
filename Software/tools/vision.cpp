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

#include "../common/coms.hpp"

#define NODE_NAME "VISION"
#define LIBCAMERA_PIPE "libcamera-vid -t 0 --camera 0 --nopreview --autofocus-mode manual --codec yuv420 --width 640 --height 360 --inline --listen -o -"
#define OUTPUT_PIPE "///tmp/video_out"

const int WIDTH = 640;
const int HEIGHT = 360;
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;
const int THRESHOLD = 10;

const cv::Vec3b redYUV = cv::Vec3b(81, 90, 240);
const cv::Vec3b greenYUV = cv::Vec3b(145, 54, 34);
const cv::Vec3b blueYUV = cv::Vec3b(41, 240, 110);
const cv::Vec3b yellowYUV = cv::Vec3b(210, 16, 146);


int main(int argc, char* argv[]) {

    enum {OUT_GRAY, OUT_WARP, OUT_GRAD, OUT_COST } output_mode = OUT_WARP;
    enum {MODE_RUN, CALIBRATE } mode = MODE_RUN;

    //  **************** Runtime Parameters **************** //
    // Register Parameters
    float horizon_height = HEIGHT / 2;
    float threshold_value = 63;
    float transfromPadding = 100;
    float grad_dx = 1;
    float grad_dy = 0;
    float grad_ksize = 3;
    float grad_scale = 1;
    float grad_delta = 0;
    float nextOutput = 0;
    float nextMode = 0;
    // --------- Communication Setup --------------------- //
    Protocol proto = {'<', '\n', ':', 'A'};
    Command cmd; // Working command variable
    std::string cmd_msg; // Command message string
    std::string cmdret_msg; // Command return message string
    TelemetryMsg telem; // Working telemetry variable
    NodeConfigManager config("robotConfig.json", NODE_NAME);
    config.register_parameter(0,&horizon_height);
    config.register_parameter(1,&threshold_value);
    config.register_parameter(2,&transfromPadding);
    config.register_parameter(3,&grad_dx);
    config.register_parameter(4,&grad_dy);
    config.register_parameter(5,&grad_ksize);
    config.register_parameter(6,&grad_scale);
    config.register_parameter(7,&grad_delta);
    config.register_parameter(8,&nextOutput);
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
    // Define source and destination points for perspective transform
    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(0, HEIGHT), cv::Point2f(WIDTH, HEIGHT), cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)};
    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(WIDTH / 2 - transfromPadding, HEIGHT),
        cv::Point2f(WIDTH / 2 + transfromPadding, HEIGHT),
        cv::Point2f(0, 0),
        cv::Point2f(WIDTH, 0)};
    // Precompute the perspective transform matrix
    cv::Mat transform_matrix = cv::getPerspectiveTransform(src_pts, dst_pts);
    
    // ------------- Console Preprocessing ----------------- //
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
        @note: Outputs frame to video out pipe 
        */
        if (poll_items[0].revents & ZMQ_POLLIN) { // Frame Available
            // Read raw YUV420 frame from the pipe
            size_t bytes_read = fread(yuv_buffer.data(), 1, FRAME_SIZE, in_pipe);
            if (bytes_read != FRAME_SIZE) {
                std::cerr << "Error reading frame from rpicam-vid!" << std::endl;
                break;
            }
            // Fill U and V planes with constant values (e.g., 128)
            uint8_t* u_plane = yuv_buffer.data() + (WIDTH * HEIGHT);
            uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);
            std::fill(u_plane, u_plane + (WIDTH * HEIGHT / 4), 128);
            std::fill(v_plane, v_plane + (WIDTH * HEIGHT / 4), 128);

            // Process Y plane
            cv::Mat y_plane(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data());
            cv::Mat roi = y_plane(cv::Rect(0, HEIGHT - horizon_height, WIDTH, horizon_height));

            // Apply perspective transform to the ROI
            cv::Mat transformed_roi;
            cv::warpPerspective(roi, transformed_roi, transform_matrix, roi.size());

            // Apply Sobel filter to the transformed ROI
            cv::Mat sobel_image;
            cv::Sobel(transformed_roi, sobel_image, CV_8U, grad_dx, grad_dy, grad_ksize, grad_scale, grad_delta, cv::BORDER_DEFAULT);

            // Apply Distance Transform
            cv::Mat distance_transform;
            cv::distanceTransform(sobel_image, distance_transform, cv::DIST_C, cv::DIST_MASK_PRECISE);
            cv::normalize(distance_transform, distance_transform, 0, 1.0, cv::NORM_MINMAX);

            // Choose output frame based on the mode
            cv::Mat* output_frame;
            switch (output_mode) {
                case OUT_GRAY:
                    output_frame = &y_plane;
                    break;
                case OUT_WARP:
                    output_frame = &transformed_roi;
                    break;
                case OUT_GRAD:
                    output_frame = &sobel_image;
                    break;
                case OUT_COST:
                    output_frame = &distance_transform;
                    break;
                default:
                    output_frame = &y_plane;
                    break;
            }
            // Resize the image to 640x360
            cv::resize(*output_frame, *output_frame, cv::Size(WIDTH, HEIGHT));
            std::memcpy(yuv_buffer.data(), output_frame->data, WIDTH * HEIGHT);
            fwrite(yuv_buffer.data(), 1, FRAME_SIZE, out_pipe);
        }
        
        // ------- Command Processing -------- //
        if (poll_items[1].revents & ZMQ_POLLIN) { // Console Input
            handle_cmdconsole(cmd, cmd_pubsock, config, proto);
        }
        if (poll_items[2].revents & ZMQ_POLLIN) { // Command Message Available
            handle_zmqcmd(cmd, cmd_subsock, msg_pubsock, config, proto);
        }
    }
    pclose(in_pipe);
    fclose(out_pipe);
    return 0;
}
