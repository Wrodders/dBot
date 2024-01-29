#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

int main() {
    // Create a GStreamer pipeline string for UDP streaming
   
   
    // Create a VideoCapture object with GStreamer pipeline
    cv::VideoCapture cap("udp://127.0.0.1:5000", cv::CAP_FFMPEG);

    // Check if the VideoCapture object is opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream!" << std::endl;
        return -1;
    }

    // Create a window for displaying the video stream
    cv::namedWindow("GStreamer Video Stream", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;

        // Read a frame from the video stream
        cap >> frame;

        // Check if the frame is empty
        if (frame.empty()) {
            std::cerr << "End of video stream" << std::endl;
            break;
        }

        // Display the frame in the window
        cv::imshow("GStreamer Video Stream", frame);

        // Break the loop if 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Release the VideoCapture object and close the window
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
