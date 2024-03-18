#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char** argv) {
    cv::VideoCapture cap;

    // Check if command-line arguments are provided
    if (argc > 1) {
        std::string source = argv[1];
        fprintf(stdout, "Opening Video Stream: %s", source.c_str());
        cap.open(source); // open Source
    }
    else {cap.open(0);} // Open WebCam
    if (!cap.isOpened()) {
            fprintf(stderr,"Error opening video stream:");
            return -1;
        }

    // Create a window for displaying the video stream
    cv::namedWindow("Video Stream", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;
        cap >> frame; // Read a frame from the video stream
        if (frame.empty()) {
            fprintf(stderr,"End of video stream");
            break;
        }
        cv::imshow("Video Stream", frame);

        // Break the loop if esc is pressed
        if (cv::waitKey(1) =='q') {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
