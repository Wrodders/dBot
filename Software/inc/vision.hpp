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


struct state{
    double pitch; // degrees
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
};
struct ROIData {
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
ROIData roiData;

void drawGridLines(cv::Mat& image, int numDivisions, int divisionHeight) {
    // vertical and horizontal lines
    int rows = image.rows;
    int cols = image.cols;
    for (int i = 1; i < numDivisions; ++i) {
        cv::line(image, cv::Point(0, i * divisionHeight), cv::Point(cols, i * divisionHeight), cv::Scalar(255, 255, 255), 1);
        cv::line(image, cv::Point(i * divisionHeight, 0), cv::Point(i * divisionHeight, rows), cv::Scalar(255, 255, 255), 1);
    }
}



void selectROI(int event, int x, int y, int flags, void* param) {
    auto* data = static_cast<ROIData*>(param);

    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
            // If 4 points are selected, reset the points and start over
            if (data->pointCount == 4) {
                data->points.clear();  // Clear previous points
                data->pointCount = 0;  // Reset point counter
                data->roiSelected = false;  // Reset ROI selection
            }

            if (data->pointCount < 4) {  // Allow selecting up to 4 points
                data->points.push_back(cv::Point(x, y));
                data->pointCount++;
            }
            // If 4 points are selected, mark the ROI as selected
            if (data->pointCount == 4) {
                data->roiSelected = true;
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            // Optional: Visual feedback for drawing (if needed)
            break;

        case cv::EVENT_LBUTTONUP:
            break;

        default:
            break;
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
struct TargetVelocities method2(cv::Mat& frame, cv::Mat& result, ROIData roiData, struct state state) {
    struct TargetVelocities targetVelocities = {0.0, 0.0};

    // Synchronize pitch with camera timestamp
    double currentPitch = state.pitch;  // Get the current pitch value

    if (roiData.roiSelected && roiData.pointCount == 4) {
        // Define the source points (the corners of the ROI)

        // Apply ROI with padding on the transformed result if necessary
        cv::Mat roi;
        applyROIToFrameWithPadding(frame, roiData.points, roi, 10, 10);

        // Preprocess ROI
        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        cv::threshold(roi, roi, threshold_value, 255, cv::THRESH_BINARY);
        cv::GaussianBlur(roi, roi, cv::Size(5, 5), 0);



        // split into 3x3 grid 
        int numDivisions = 9;
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
        // draw a cirle with the pitch offset applied to the center centorid
        cv::circle(roi, cv::Point(cx, cy+kd*state.pitch), 5, cv::Scalar(0,255,0 ), -1);

        // Draw the centroid of each grid
        for (int i = 0; i < numDivisions; ++i) {
            cv::circle(roi, cv::Point(gridCentroids[i], i * divisionHeight + divisionHeight / 2), 5, cv::Scalar(255, 0, 0), -1);
        }

        // Compute the target velocities

        result = roi;

    }

    return targetVelocities;
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


void onThresholdSlider(int, void*) {
    std::cout << "Threshold: " << threshold_value << std::endl;
}

void onKPSlider(int, void*) {
    std::cout << "KP: " << AP << std::endl;
}

void onKISlider(int, void*) {
    std::cout << "KI: " << KI << std::endl;
}

void onKDSlider(int, void*) {
    std::cout << "KD: " << kd << std::endl;
}

void onVelScaleSlider(int, void*) {
    std::cout << "Velocity Scale: " << velScale << std::endl;
}




void setupZMQPublisher(zmq::context_t &context, zmq::socket_t &publisher) {
    publisher.bind("tcp://*:5556");  // Binding to TCP socket on port 5556
    std::cout << "Publisher bound to tcp://*:5556\n";
}

void sendZMQMessage(zmq::socket_t &publisher, char id, double val) {
    // Define the topic
    std::string topic = "SERIAL";
    std::string message = "<b" + std::string(1, id)+ std::to_string(val)+"\n";

    zmq::message_t topicMessage(topic.c_str(), topic.size());
    zmq::message_t zmqMessage(message.size());
    memcpy(zmqMessage.data(), message.c_str(), message.size());
    publisher.send(topicMessage, zmq::send_flags::sndmore);
    publisher.send(zmqMessage, zmq::send_flags::none);
}








#endif // VISION_H