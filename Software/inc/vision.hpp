#ifndef VISION_H
#define VISION_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>

#include <thread>
#include <zmq.hpp> // Include the ZeroMQ header
#include <getopt.h>  // For command-line argument parsing

// Main method for processing frame and printing centroid data//
// return the target linear and angular velocities
struct TargetVelocities {
    double linearVelocity;
    double angularVelocity;
};


struct ROIinfo {
    std::vector<cv::Point> points;  // Store selected points for the trapezium
    bool drawing = false;
    bool roiSelected = false;
    int pointCount = 0;  // Counter to track the number of points selected
};
// Global flags and variables
bool stopFlag = false;
int threshold_value = 63;
int AP = 100;
int KI = 0;
int kd = 0;  
int velScale = 0;
ROIinfo roiData;

void drawGridLines(cv::Mat& image, int numDivisions, int divisionHeight) {
    // vertical and horizontal lines
    int rows = image.rows;
    int cols = image.cols;
    for (int i = 1; i < numDivisions; ++i) {
        cv::line(image, cv::Point(0, i * divisionHeight), cv::Point(cols, i * divisionHeight), cv::Scalar(255, 255, 255), 1);
        cv::line(image, cv::Point(i * divisionHeight, 0), cv::Point(i * divisionHeight, rows), cv::Scalar(255, 255, 255), 1);
    }
}


void applyROIToFrameWithPadding(cv::Mat& frame, const std::vector<cv::Point>& roiPoints, cv::Mat& result, int topPadding, int bottomPadding) {
    // Calculate the bounding rectangle for the points
    cv::Rect boundingRect = cv::boundingRect(roiPoints);

    // Adjust the bounding rectangle with separate padding values
    int adjustedTop = std::max(boundingRect.y - topPadding, 0);
    int adjustedBottom = std::min(boundingRect.y + boundingRect.height + bottomPadding, frame.rows);
    int adjustedLeft = std::max(boundingRect.x, 0);
    int adjustedRight = std::min(boundingRect.x + boundingRect.width, frame.cols);

    cv::Rect adjustedROI(adjustedLeft, adjustedTop, adjustedRight - adjustedLeft, adjustedBottom - adjustedTop);

    // Create a mask for the ROI
    cv::Mat mask = cv::Mat::zeros(adjustedROI.size(), CV_8UC1);
    std::vector<cv::Point> shiftedPoints;
    for (const auto& point : roiPoints) {
        shiftedPoints.emplace_back(point.x - adjustedROI.x, point.y - adjustedROI.y);
    }
    cv::fillPoly(mask, {shiftedPoints}, cv::Scalar(255));

    // Crop the frame and apply the mask
    cv::Mat croppedFrame = frame(adjustedROI);
    croppedFrame.copyTo(result, mask);
}
void processFrame(cv::Mat& frame, cv::Mat& result, ROIinfo roiData) {
    if (roiData.roiSelected && roiData.pointCount == 4) {
        // Define the source points (the corners of the ROI)

        // Apply ROI with padding on the transformed result if necessary
        cv::Mat roi;
        applyROIToFrameWithPadding(frame, roiData.points, roi, 10, 10);

        // Preprocess ROI
        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        cv::threshold(roi, roi, threshold_value, 255, cv::THRESH_BINARY);
        cv::GaussianBlur(roi, roi, cv::Size(5, 5), 0);
        method1(roi, result);
    }
}

void method1(cv::Mat &roi, cv::Mat &result) {
// split into 3x3 grid 
    int numDivisions = numDivSliderValue;
    int divisionHeight = roi.rows / numDivisions;
    // Compute the centroid of the binary image
    cv::Moments moments = cv::moments(roi, true);
    double cx = moments.m10 / moments.m00;
    double cy = moments.m01 / moments.m00;

    // compute sentroid for each grid
    // Compute the centroid of each grid
    double gridCentroids[numDivisions];
    for (int i = 0; i < numDivisions; ++i) {
        cv::Mat grid = roi(cv::Rect(0, i * divisionHeight, roi.cols, divisionHeight));
        cv::Moments gridMoments = cv::moments(grid, true);
        gridCentroids[i] = gridMoments.m10 / gridMoments.m00;
    }

    cv::cvtColor(roi, roi, cv::COLOR_GRAY2BGR);
    drawGridLines(roi, numDivisions, divisionHeight);
    // Draw the centroid on the result image
    cv::circle(roi, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);

    // Draw the centroid of each grid
    for (int i = 0; i < numDivisions; ++i) {
        cv::circle(roi, cv::Point(gridCentroids[i], i * divisionHeight + divisionHeight / 2), 5, cv::Scalar(255, 0, 0), -1);
    }
    result = roi;
}


void zmqPublishWorker(zmq::socket_t& publisher) {
    while (!stopFlag) {
        // Sleep to maintain 30Hz sending rate
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Lock mutex to access the latest velocity data safely
        std::lock_guard<std::mutex> lock(velocityMutex);
        
        // Send the latest angular velocity (multiplied by kernel size)
        sendZMQMessage(publisher, 'm',  -0.01f*velScale);
        sendZMQMessage(publisher, 'r', fabs(latestVel.angularVelocity) < 0.8 ? latestVel.angularVelocity : 0.0);

        sendZMQMessage(publisher, 's', (float)AP / 5.0  );
        sendZMQMessage(publisher, 't', (float)KI / 5.0 );
        sendZMQMessage(publisher, 'u', (float)kd / 5.0 );
    }
}
void zmqSubscriberWorker(zmq::socket_t& subscriber) {
    // receive multipart message and update the state with new valeu and timestamp
    // block thread until message is received

    std::cout << "Starting ZMQ Subscriber\n";

    while (!stopFlag) {
        zmq::message_t topic_msg;
        zmq::message_t data_msg;
        int rc = *subscriber.recv(topic_msg, zmq::recv_flags::none);
        if (rc) {
            std::string topic = std::string(static_cast<char*>(topic_msg.data()), topic_msg.size());
            rc = *subscriber.recv(data_msg, zmq::recv_flags::none);
            if (rc) {
                std::string data = std::string(static_cast<char*>(data_msg.data()), data_msg.size());
                state.pitch = std::stof(data);
                state.timestamp = std::chrono::steady_clock::now();
            }
        }
    }
}
#endif // VISION_H