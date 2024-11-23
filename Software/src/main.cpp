#include <opencv2/opencv.hpp>
#include <iostream>
#include "../inc/roi_selector.h"

void printHelp() {
    std::cout << "Usage: VideoROI [options]\n"
              << "Options:\n"
              << "  --source [camera|udp]       Video source (default: camera).\n"
              << "  --udp_address [IP]          UDP stream address (default: 192.168.1.122).\n"
              << "  --udp_port [PORT]           UDP stream port (default: 5000).\n"
              << "  --help                      Display this help message.\n";
}

int main(int argc, char* argv[]) {
    // Parse command-line arguments
    std::string source = "camera";
    std::string udpAddress = "192.168.1.122";
    int udpPort = 5000;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--source" && i + 1 < argc) source = argv[++i];
        else if (arg == "--udp_address" && i + 1 < argc) udpAddress = argv[++i];
        else if (arg == "--udp_port" && i + 1 < argc) udpPort = std::stoi(argv[++i]);
        else if (arg == "--help") {
            printHelp();
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printHelp();
            return -1;
        }
    }

    // Open video source
    cv::VideoCapture cap;
    if (source == "udp") {
        std::string videoSource = "udp://" + udpAddress + ":" + std::to_string(udpPort);
        cap.open(videoSource);
    } else {
        cap.open(1); // Default to camera
    }

    if (!cap.isOpened()) {
        std::cerr << "Error: Couldn't open the video stream or camera.\n";
        return -1;
    }

    // Get actual resolution
    int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout << "Camera resolution: " << actualWidth << "x" << actualHeight << "\n";

    // Set up ROI data and mouse callback
    ROIData roiData;
    cv::namedWindow("Original Video", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Original Video", selectROI, &roiData);

    // Create a window for the thresholded ROI
    cv::namedWindow("Thresholded ROI", cv::WINDOW_NORMAL);

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "Error: Couldn't read frame.\n";
            break;
        }

        // Draw ROI rectangle on the original frame
        if (roiData.drawing || roiData.roiSelected) {
            cv::rectangle(frame, roiData.roi, cv::Scalar(0, 255, 0), 2);
        }

        // Display the original frame
        cv::imshow("Original Video", frame);

        // Process ROI if selected
        if (roiData.roiSelected && roiData.roi.width > 0 && roiData.roi.height > 0) {
            cv::Mat roiFrame = frame(roiData.roi);
            cv::Mat grayROI, thresholdedROI;

            // Convert ROI to grayscale
            cv::cvtColor(roiFrame, grayROI, cv::COLOR_BGR2GRAY);
            // Apply dynamic threshold
            cv::threshold(grayROI, thresholdedROI, 56, 255, cv::THRESH_BINARY);

            // Display the thresholded ROI
            cv::imshow("Thresholded ROI", thresholdedROI);
        }

        // Check for 'q' key press
        int key = cv::waitKey(10);  // Wait for 10 ms and capture key events
        if (key == 'q' || key == 27) {  // 27 is the ASCII for 'Esc' key
            break;
        }
    }

    // Print ROI information
    if (roiData.roiSelected) {
        std::cout << "Selected ROI: x=" << roiData.roi.x << ", y=" << roiData.roi.y
                  << ", width=" << roiData.roi.width << ", height=" << roiData.roi.height << "\n";
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
