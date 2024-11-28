#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <zmq.hpp> // Include the ZeroMQ header
#include <getopt.h>  // For command-line argument parsing


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

// Default values
std::string udpAddress = "udp://localhost:5000";
std::string outputFile = "output.mp4";

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
        //std::cerr << "Error: No data received from FFmpeg process.\n";
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

// Function to send the message under the topic SERIAL
void sendZMQMessage(zmq::socket_t &publisher, double val) {
    // Define the topic
    std::string topic = "SERIAL";

    // Format the message: <bm value>
    std::string message = "<br" + std::to_string(val*kernelSize/10) + "\n";

    // Create a ZMQ message for the topic
    zmq::message_t topicMessage(topic.c_str(), topic.size());

    // Create a ZMQ message for the message content
    zmq::message_t zmqMessage(message.size());
    memcpy(zmqMessage.data(), message.c_str(), message.size());

    // Send the topic message first, followed by the actual message
    publisher.send(topicMessage, zmq::send_flags::sndmore);  // Topic with `sndmore` flag
    publisher.send(zmqMessage, zmq::send_flags::none);       // Message with `none` flag

    // Print the message sent (for debugging)
    //td::cout << "Sent topic: " << topic << " message: " << message << std::endl;
}

// Global variable to store the estimated angle
std::atomic<float> globalEstimatedAngle(0.0f);

// Function to send ZMQ messages at 10 Hz
void zmqSender(zmq::socket_t& publisher) {
    while (true) {
        float angle = globalEstimatedAngle.load();
        sendZMQMessage(publisher, angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }
}

void method1(cv::Mat& frame, cv::Mat& result, ROIData roiData, zmq::socket_t& publisher) {
    if (roiData.roiSelected && roiData.pointCount == 4) {
        cv::Mat roi;
        applyROIToFrameWithPadding(frame, roiData.points, roi, 10);
        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        cv::threshold(roi, result, threshold_value, 255, 0);
        cv::GaussianBlur(result, result, cv::Size(29, 29), 0);
        cv::morphologyEx(result, result, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25)), cv::Point(-1, -1), morphIterations);
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

        // Fit lines to the points
        std::vector<cv::Point> points;
        fitLine(result, points, 1, numSegments, 10, 200, 0);

        // Compute the angle of the lines for each segment into an array 
        std::vector<float> angles;
        for (size_t i = 1; i < points.size(); ++i) {
            float angle = calculateAngle(points[i - 1], points[i]);
            angles.push_back(angle);
        }

        // Compute average angle of the line. Normalize this to -1 and 1 for issuing angular velocity commands to the robot 
        float estimatedAngle = 0;
        for (size_t i = 0; i < angles.size(); ++i) {
            estimatedAngle += angles[i];
        }
        estimatedAngle = estimatedAngle / angles.size();


        // Update the global estimated angle
        globalEstimatedAngle.store(estimatedAngle);
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
            //std::cerr << "Error reading frame\n";
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Allow some time for the next frame to be ready
    }
}


// Function to display help text
void displayHelp() {
    std::cout << "Usage: program_name [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --help           Display this help message\n";
    std::cout << "  --source <path>  Specify the source file (default: videoData.mp4)\n";
}

// Function to parse the arguments
std::string handleCLI(int argc, char* argv[]) {
    std::string source = "videoData.mp4";  // Default value for source file
    
    // Check for --help flag
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            displayHelp();
            exit(0);  // Exit after showing help
        }
    }

    // Parse --source flag and its argument
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--source" && i + 1 < argc) {
            source = argv[i + 1];
            i++;  // Skip next argument, it's the source file path
        }
    }
    
    return source;
}

int main(int argc, char* argv[]) {
    // Parse the command-line arguments
    udpAddress = handleCLI(argc, argv);
    // ZMQ publisher setup
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    setupZMQPublisher(context, publisher);

    std::thread zmqThread(zmqSender, std::ref(publisher));

    if (!openFFmpegProcess(udpAddress)) {
            std::cerr << "Error: Could not open UDP stream.\n";
            return -1;
        }

    cv::namedWindow("Original Video", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Original Video", selectROI, &roiData);
    setupTrackBars();

    cv::Mat frame, resultFrame;
    std::thread readThread(readFramesAsync, std::ref(frame));

    while (!stopFlag) {
        if (frameReady) {
            method1(frame, resultFrame, roiData, publisher);
            if (!resultFrame.empty()) {
                cv::imshow("Processed Video", resultFrame);
            }
            cv::imshow("Original Video", frame);
            frameReady = false; // Reset the frame flag
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            stopFlag = true;
            break;
        }
    }

    // Clean up
    if (ffmpegProcess) {
        fclose(ffmpegProcess);
    }

    cv::destroyAllWindows();
    readThread.join();
    zmqThread.join();

    return 0;
}