#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <zmq.hpp> // Include the ZeroMQ header

#include "../inc/roi_selector.h"
#include "../inc/utils.h"
#include "../inc/fitline.h"

// Global flags and variables
bool stopFlag = false;
int threshold_value = 63;
int numSegments = 9;
int kernelSize = 25;
int morphIterations = 2;  // Default iterations for closing

// FFmpeg process handling variables
FILE* ffmpegProcess = nullptr;
std::atomic<bool> frameReady(false);
std::atomic<int> frameDropCount(0);

ROIData roiData;

// Set the correct resolution based on the stream
const int frameWidth = 640;  // Adjust to the stream's width (e.g., 640x480)
const int frameHeight = 480; // Adjust to the stream's height (e.g., 640x480)

// Open FFmpeg process to pipe video (with proper command)
bool openFFmpegProcess(const std::string& streamUrl) {
    std::string command = "ffmpeg -i " + streamUrl + " -f rawvideo -pix_fmt bgr24 -vsync 0 -an -sn -fflags nobuffer -flags low_delay pipe:1";

    // Open the FFmpeg process for reading its output
    ffmpegProcess = popen(command.c_str(), "r");
    if (!ffmpegProcess) {
        std::cerr << "Error: Could not open FFmpeg process.\n";
        return false;
    }

    return true;
}

// Read frame from FFmpeg pipe
bool readFFmpegFrame(cv::Mat& frameMat) {
    if (!ffmpegProcess) return false;

    // Calculate the frame size based on 640x480 resolution
    size_t frameSize = frameWidth * frameHeight * 3;  // 3 channels (BGR)

    // Create the Mat object for the frame
    frameMat.create(frameHeight, frameWidth, CV_8UC3);

    // Read raw frame data from the FFmpeg process
    size_t bytesRead = fread(frameMat.data, 1, frameSize, ffmpegProcess);

    if (bytesRead == 0) {
        std::cerr << "Error: No data received from FFmpeg process.\n";
        return false;
    }

    if (bytesRead != frameSize) {
        std::cerr << "Warning: Incomplete frame read. Expected " << frameSize << " bytes, but got " << bytesRead << " bytes.\n";
        return false;
    }

    frameReady = true; // Mark frame as ready for processing
    return true;
}


void setupZMQPublisher(zmq::context_t &context, zmq::socket_t &publisher) {
    publisher.bind("tcp://*:5556");  // Binding to TCP socket on port 5556
    std::cout << "Publisher bound to tcp://*:5556\n";
}

// Function to send the message
// Function to send the message
void sendZMQMessage(zmq::socket_t &publisher, double val) {
    // Format the message: <bm value>
    std::string message = "<bm" + std::to_string(val) + "\n";

    // Create a ZMQ message (multipart message with one part)
    zmq::message_t zmqMessage(message.size());
    memcpy(zmqMessage.data(), message.c_str(), message.size());

    // Send the message as a multipart message (one part for this case)
    publisher.send(zmqMessage, zmq::send_flags::none);
    std::cout << "Sent message: " << message << std::endl;
}
// Update method1 to include ZMQ publisher for sending brightness difference
void method1(cv::Mat& frame, cv::Mat& result, ROIData roiData, zmq::socket_t &publisher) {
    if (roiData.roiSelected && roiData.pointCount == 4) {
        cv::Mat roi;
        applyROIToFrameWithPadding(frame, roiData.points, roi, 10);
        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        cv::threshold(roi, result, threshold_value, 255, 0);
        cv::GaussianBlur(result, result, cv::Size(29, 29), 0);
        cv::morphologyEx(result, result, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25)), cv::Point(-1, -1), morphIterations);
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

        // Process and analyze lines
        int halfWidth = roi.cols / 2;
        cv::Mat leftHalf = roi(cv::Rect(0, 0, halfWidth, roi.rows));
        cv::Mat rightHalf = roi(cv::Rect(halfWidth, 0, roi.cols - halfWidth, roi.rows));

        double leftAvgBrightness = cv::mean(leftHalf)[0];
        double rightAvgBrightness = cv::mean(rightHalf)[0];

        double brightnessDifference = (leftAvgBrightness - rightAvgBrightness) / (leftAvgBrightness + rightAvgBrightness);

        std::cout << "Brightness difference between left and right halves: " << brightnessDifference << std::endl;

        // Send the message with brightness difference using ZMQ
        sendZMQMessage(publisher, brightnessDifference);
    }
}

// Update ROI (Region of Interest) display
void updateROI(cv::Mat& frame, ROIData roiData) {
    if (roiData.pointCount > 0) {
        for (size_t i = 0; i < roiData.points.size(); ++i) {
            cv::circle(frame, roiData.points[i], 5, cv::Scalar(0, 0, 255), -1);
        }
        if (roiData.pointCount == 4) {
            cv::polylines(frame, roiData.points, true, cv::Scalar(0, 255, 0), 2);
        }
    }
}

// Setup trackbars for adjusting parameters
void setupTrackBars() {
    cv::createTrackbar("Threshold", "Original Video", &threshold_value, 255);
    cv::createTrackbar("Num Segments", "Original Video", &numSegments, 25);
    cv::createTrackbar("Kernel Size", "Original Video", &kernelSize, 70);
    cv::createTrackbar("Morph Iterations", "Morphologically Closed", &morphIterations, 10);
}

// Thread to handle frame reading from FFmpeg process
void readFramesAsync(cv::Mat& frame) {
    while (!stopFlag) {
        if (!readFFmpegFrame(frame)) {
            std::cerr << "Error reading frame\n";
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Allow some time for the next frame to be ready
    }
}

// Main function
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " --source <udp> [--udp_address <address>] [--udp_port <port>]\n";
        return -1;
    }



    std::string tcpAddress = "tcp://*:5556";  // Example UDP stream address
    // ZMQ: Send the message with brightness difference
    // Create a ZeroMQ context and socket (publisher)
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    setupZMQPublisher(context, publisher);

    std::string udpAddress = "udp://localhost:5000";  // Example UDP stream address
    // Open FFmpeg process to stream video
    if (!openFFmpegProcess(udpAddress)) {
        return -1;
    }

    cv::namedWindow("Original Video", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Original Video", selectROI, &roiData);  // Assuming selectROI is defined
    setupTrackBars();

    cv::Mat frame, resultFrame;

    std::thread readThread(readFramesAsync, std::ref(frame));
    while (!stopFlag) {
        // Process at 50Hz (i.e., every 20ms)
        if (frameReady) {
            // Process the frame with ROI and other methods
            method1(frame, resultFrame, roiData, publisher);
            updateROI(frame, roiData);
 


            if (!resultFrame.empty()) {
                cv::imshow("Processed Video", resultFrame);
            }
            cv::imshow("Original Video", frame);

            frameReady = false;  // Reset frame flag
        }
     
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            stopFlag = true;
            break;
        }
    }

    // Cleanup and close FFmpeg process
    if (ffmpegProcess) {
        fclose(ffmpegProcess);
    }

    cv::destroyAllWindows();
    readThread.join();  // Wait for the reading thread to finish

    return 0;
}
