#include <iostream>
#include <cstdio>
#include <vector>

const int WIDTH = 640;  // 640x360 resolution
const int HEIGHT = 360;
const int FRAME_SIZE = WIDTH * HEIGHT * 3 / 2; // YUV420p (Y + U + V)

const int THRESHOLD = 100;  // Threshold value for binary image

int main() {
    // Open the pipe to rpicam-vid command (YUV420p output)
    const char* rpicam_cmd = "rpicam-vid -t 0 --camera 0 --rotation 180 --nopreview --codec yuv420 --width 640 --height 360 --inline --listen -o -";
    FILE* pipe = popen(rpicam_cmd, "r");
    if (!pipe) {
        std::cerr << "Failed to open pipe to rpicam-vid!" << std::endl;
        return -1;
    }

    // Allocate a buffer once (no need to reallocate every time)
    std::vector<uint8_t> yuv_buffer(FRAME_SIZE);

    // Pre-allocate OpenCV matrices for YUV frame and the thresholded Y plane
    uint8_t* y_plane = &yuv_buffer[0];
    uint8_t* u_plane = &yuv_buffer[WIDTH * HEIGHT];
    uint8_t* v_plane = u_plane + (WIDTH * HEIGHT / 4);

    // Loop to continuously read, process, and write frames
    while (true) {
        // Read raw YUV420 frame from rpicam-vid pipe
        size_t bytes_read = fread(yuv_buffer.data(), 1, FRAME_SIZE, pipe);
        if (bytes_read != FRAME_SIZE) {
            std::cerr << "Error reading frame from rpicam-vid!" << std::endl;
            break;
        }

        // Process the Y (luminance) plane: Apply thresholding to create a binary image
        // In this case, we're simply thresholding the Y plane to create a simple binary image.
        for (int i = 0; i < HEIGHT; i++) {
            for (int j = 0; j < WIDTH; j++) {
                uint8_t& pixel = y_plane[i * WIDTH + j];
                pixel = (pixel > THRESHOLD) ? 255 : 0;  // Threshold the pixel
            }
        }

        // Fill the U and V planes with constant values (e.g., 128)
        // Since we're working with grayscale, U and V are just placeholders
        
        for (int i = 0; i < (HEIGHT / 2); i++) {
            for (int j = 0; j < (WIDTH / 2); j++) {
                u_plane[i * (WIDTH / 2) + j] = 128;
                v_plane[i * (WIDTH / 2) + j] = 128;
            }
        }
        
        // Stream the modified YUV420p frame to stdout for ffmpeg
        fwrite(yuv_buffer.data(), 1, FRAME_SIZE, stdout);
    }

    // Cleanup and close the pipe
    fclose(pipe);

    return 0;
}
