#include <iostream>
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp> // OpenCV library

const int WIDTH = 640;   // Frame width
const int HEIGHT = 360;  // Frame height
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2; // YUV420p (Y + U + V)

const int THRESHOLD = 50;  // Threshold value for binary image



int main() {
    // Command to capture video using rpicam-vid with YUV420p output
    const char* rpicam_cmd = "rpicam-vid -t 0 --camera 0 --rotation 180 --nopreview --codec yuv420 --width 640 --height 360 --inline --listen -o -";
    FILE* pipe = popen(rpicam_cmd, "r");
    if (!pipe) {
        std::cerr << "Failed to open pipe to rpicam-vid!" << std::endl;
        return -1;
    }

    // Allocate buffer for YUV data
    std::vector<uint8_t> yuv_buffer(FRAME_SIZE);

    // Loop to continuously read and process frames
    while (true) {
        // Read raw YUV420 frame from the pipe
        size_t bytes_read = fread(yuv_buffer.data(), 1, FRAME_SIZE, pipe);
        if (bytes_read != FRAME_SIZE) {
            std::cerr << "Error reading frame from rpicam-vid!" << std::endl;
            break;
        }

        // Map YUV420 data to OpenCV matrices
        cv::Mat yuv_image(HEIGHT + HEIGHT / 2, WIDTH, CV_8UC1, yuv_buffer.data());
        cv::Mat y_plane(HEIGHT, WIDTH, CV_8UC1, yuv_buffer.data());

        // Threshold the Y plane to create a binary image
        cv::Mat binary_image;
        cv::threshold(y_plane, binary_image, THRESHOLD, 255, cv::THRESH_BINARY);

        // canny edge detection
        cv::Mat edges;
        cv::Canny(binary_image, edges, 100, 200);
        // Fill U and V planes with constant values (e.g., 128)
        uint8_t* u_plane = yuv_buffer.data() + (WIDTH * HEIGHT);
        uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);
        std::fill(u_plane, u_plane + (WIDTH * HEIGHT / 4), 128);
        std::fill(v_plane, v_plane + (WIDTH * HEIGHT / 4), 128);

        // Replace the Y plane in the original YUV buffer with the binary image
        std::memcpy(yuv_buffer.data(), edges.data, WIDTH * HEIGHT);

        // Stream the modified YUV420p frame to stdout
        fwrite(yuv_buffer.data(), 1, FRAME_SIZE, stdout);
    }

    // Cleanup and close the pipe
    fclose(pipe);
    return 0;
}
