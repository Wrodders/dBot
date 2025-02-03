#include <iostream>
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp> 
#include <zmq.hpp>

#include "../inc/twsbDriver.hpp"

/************ OpenCV Camera Pipeline Filter Stage ***************************/
/*
|------------|--------|---------|--------|--------|
| RPICAM-VID | yuv420 | camFilt | yuv420 | ffmpeg |
*/

const int WIDTH = 640;   // Frame width
const int HEIGHT = 360;  // Frame height
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2; // YUV420p (Y + U + V)

const int THRESHOLD = 10;  // Threshold value for binary image
int main() {
    // Command to capture video using rpicam-vid with YUV420p output
    const char* rpicam_cmd = "rpicam-vid -t 0 --camera 0 --rotation 180  --autofocus-mode manual --nopreview --codec yuv420 --width 640 --height 360 --inline --listen -o -";
    FILE* pipe = popen(rpicam_cmd, "r");
    if (!pipe) {
        std::cerr << "Failed to open pipe to rpicam-vid!" << std::endl;
        return -1;
    }

    // ***************** Parameters     
    enum Parameters {P_HORIZON, P_THRESHOLD, P_TRANSFORM, P_GRAD, P_OUTPUT, NUM_PARAMS};
    int horizon_height =HEIGHT/2; // Horizon height for the perspective transform
    int threshold_value = 63;
    int transfromPadding = 100; // Padding for perspective transform
    int grad_dx = 1; // Order of the derivative x
    int grad_dy = 0; // Order of the derivative y
    int grad_ksize = 3; // Size of the extended Sobel kernel
    double grad_scale = 1; // Optional scale factor for the computed derivative values
    double grad_delta = 0; // Optional delta value that is added to the results prior to storing them
    enum {OUT_GRAY, OUT_WARP, OUT_GRAD, OUT_COST} output_mode = OUT_GRAD;

   

    // create a zmq subscriber that listens for commands non blocking 
    zmq::context_t context(1);
    zmq::socket_t subSocket = zmq::socket_t(context, zmq::socket_type::sub);
    subSocket.set(zmq::sockopt::linger, 0);
    subSocket.connect("ipc:///tmp/botCMDS"); // General Bot Commands Socket
    subSocket.set(zmq::sockopt::subscribe, "VISON"); // Receive Commands for the vision system   
    //** ZMQ Command Server **//
    // Allocate buffer for YUV data
    std::vector<uint8_t> yuv_buffer(FRAME_SIZE);
    // apply birds eue view trasfrom 
    // Define source and destination points for perspective transform
    // coordinates are in (x, y) format

    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(0, HEIGHT),  // Bottom-left
        cv::Point2f(WIDTH, HEIGHT),  // Bottom-right
        cv::Point2f(0,0),  // Top-right
        cv::Point2f(WIDTH, 0)  // Top-left
    };
    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(WIDTH/2 - transfromPadding, HEIGHT),  // Bottom-left
        cv::Point2f(WIDTH/2 + transfromPadding, HEIGHT),  // Bottom-right
        cv::Point2f(0,0),  // Top-right
        cv::Point2f(WIDTH, 0)  // Top-left
    };

    // Compute the perspective transform matrix
    cv::Mat transform_matrix = cv::getPerspectiveTransform(src_pts, dst_pts);
   
    // Loop to continuously read and process frames
    while (true) {

        // Check for any incoming commands
        zmq::message_t topic, msg;
        subSocket.recv(topic, zmq::recv_flags::dontwait);
        subSocket.recv(msg, zmq::recv_flags::dontwait);

        std::string topicStr = std::string(static_cast<char*>(topic.data()), topic.size());
        std::string msgStr = std::string(static_cast<char*>(msg.data()), msg.size());
        std::cout << "Received Command: " << msgStr << std::endl;


        // Read raw YUV420 frame from the pipe
        size_t bytes_read = fread(yuv_buffer.data(), 1, FRAME_SIZE, pipe);
        if (bytes_read != FRAME_SIZE) {
            std::cerr << "Error reading frame from rpicam-vid!" << std::endl;
            break;
        }
        // Fill U and V planes with constant values (e.g., 128)
        uint8_t* u_plane = yuv_buffer.data() + (WIDTH * HEIGHT); // U plane offset
        uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);      // V plane offset
        std::fill(u_plane, u_plane + (WIDTH * HEIGHT / 4), 128);
        std::fill(v_plane, v_plane + (WIDTH * HEIGHT / 4), 128);
        // Y Frame is luminance (intensity) of the image - grayscale
        cv::Mat y_plane(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data()); // Y plane

        // take the bottom half of the image determinted by the horixon height
        cv::Mat roi = y_plane(cv::Rect(0, HEIGHT - horizon_height, WIDTH, horizon_height));
        
        // Apply perspective transform to the ROI
        cv::Mat transformed_roi;
        cv::warpPerspective(roi, transformed_roi, transform_matrix, roi.size());

        // Apply Sobel filter to the transformed ROI
        cv::Mat sobel_image;
        cv::Sobel(transformed_roi, sobel_image, CV_8U, grad_dx, grad_dy, grad_ksize, grad_scale, grad_delta, cv::BORDER_DEFAULT);

       // Apply Distance Transform 
        cv::Mat distance_transform;
        cv::distanceTransform(sobel_image, distance_transform, cv::DIST_C,cv::DIST_MASK_PRECISE);
        // Normalize the distance transform
        cv::normalize(distance_transform, distance_transform, 0, 1.0, cv::NORM_MINMAX);

               
        cv::Mat *result;
        switch (output_mode) {
            case OUT_GRAY:
                result = &y_plane;
                break;
            case OUT_WARP:
                result = &transformed_roi;
                break;
            case OUT_GRAD:
                result = &sobel_image;
                break;

            case OUT_COST:
                result = &distance_transform;
                break; 
            default:
                result = &y_plane;
                break;
        }


        // scale the image to 640x360
        cv::resize(*result, *result, cv::Size(WIDTH, HEIGHT));


        std::memcpy(yuv_buffer.data(), result->data, WIDTH * HEIGHT);


        // Stream the modified YUV420p frame to stdout
        fwrite(yuv_buffer.data(), 1, FRAME_SIZE, stdout);
    }

    // Cleanup and close the pipe
    fclose(pipe);
    return 0;
}
