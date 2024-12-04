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


void drawGridLines(cv::Mat& image, int numDivisions, int divisionHeight) {
    int cols = image.cols;
    for (int i = 1; i < numDivisions; ++i) {
        int y = i * divisionHeight;
        cv::line(image, cv::Point(0, y), cv::Point(cols, y), cv::Scalar(0, 255, 0), 2);  // Green grid lines
    }
}

// Function to calculate the angle of a line between two points
// Function to calculate the angle of a line between two points with reference at 90 degrees
float calculateAngle(cv::Point p1, cv::Point p2) {
    // Calculate the angle in degrees from the positive X-axis
    float angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180 / CV_PI;

    // Adjust so that 90 degrees is the reference, and angles are relative to 90 degrees
    float adjustedAngle = angle+25;

    // Return the adjusted angle
    return -adjustedAngle/50;
}

void removeEdgeAreasFromROI(cv::Mat& roiImage, const std::vector<cv::Point>& roiPoints) {
    // Create a mask with the same size as the ROI image
    cv::Mat mask = cv::Mat::zeros(roiImage.size(), CV_8UC1);  // Same size as the ROI image, black by default

    // Fill the polygon (ROI boundary) with white on the mask (representing the region)
    std::vector<std::vector<cv::Point>> contours = {roiPoints};
    cv::fillPoly(mask, contours, cv::Scalar(255));

    // Set the areas along the edge (the polygon) to black

    roiImage.setTo(cv::Scalar(255), mask);  // Black out the areas defined by the mask
}

struct ROIData {
    std::vector<cv::Point> points;  // Store selected points for the trapezium
    bool drawing = false;
    bool roiSelected = false;
    int pointCount = 0;  // Counter to track the number of points selected
};
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

// Update ROI (Region of Interest) display
void applyROIToFrameWithPadding(cv::Mat& frame, const std::vector<cv::Point>& roiPoints, cv::Mat& result, int padding) {
    // Calculate the bounding rectangle for the points
    cv::Rect boundingRect = cv::boundingRect(roiPoints);

    // Expand the bounding rectangle by the padding
    boundingRect.x = std::max(boundingRect.x - padding, 0);
    boundingRect.y = std::max(boundingRect.y - padding, 0);
    boundingRect.width = std::min(boundingRect.width + 2 * padding, frame.cols - boundingRect.x);
    boundingRect.height = std::min(boundingRect.height + 2 * padding, frame.rows - boundingRect.y);

    // Create a mask for the trapezium inside the expanded bounding rectangle
    cv::Mat mask = cv::Mat::zeros(boundingRect.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours = {roiPoints};
    
    // Shift the points to fit within the bounding rectangle
    std::vector<cv::Point> shiftedPoints;
    for (const auto& point : roiPoints) {
        shiftedPoints.emplace_back(point.x - boundingRect.x, point.y - boundingRect.y);
    }

    // Fill the trapezium area within the mask
    cv::fillPoly(mask, {shiftedPoints}, cv::Scalar(255));

    // Crop the frame using the expanded bounding rectangle
    cv::Mat croppedFrame = frame(boundingRect);

    // Apply the mask to the cropped frame
    croppedFrame.copyTo(result, mask);
}



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
    //publisher.send(topicMessage, zmq::send_flags::sndmore);  // Topic with `sndmore` flag
    //publisher.send(zmqMessage, zmq::send_flags::none);       // Message with `none` flag

    // Print the message sent (for debugging)
    //td::cout << "Sent topic: " << topic << " message: " << message << std::endl;
}


// Function to send ZMQ messages at 10 Hz
void zmqSender(zmq::socket_t& publisher) {
    while (true) {
        sendZMQMessage(publisher, 9.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }
}



void applyFilters(cv::Mat& frame, const std::vector<cv::Point>& roiPoints, cv::Mat& result) {
    cv::Mat roi;
    // Apply the region of interest (ROI) to the frame
    applyROIToFrameWithPadding(frame, roiData.points, roi, 10);
    
    // Convert to grayscale
    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
    
    // Threshold the image
    cv::threshold(roi, result, threshold_value, 255, 0);
    
    // Apply Gaussian blur
    cv::GaussianBlur(result, result, cv::Size(29, 29), 0);
    
    // Apply morphological closing
    cv::morphologyEx(result, result, cv::MORPH_CLOSE,
                      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25)), cv::Point(-1, -1), morphIterations);
    
    // Convert back to color (to be used in `method1` or `method2`)
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
}

void method1(cv::Mat& frame, cv::Mat& result, ROIData roiData, zmq::socket_t& publisher) {
    if (roiData.roiSelected && roiData.pointCount == 4) {
        // Apply filtering steps using the helper function
        applyFilters(frame, roiData.points, result);

        // Fit lines to the points (example of how you might fit lines)
        std::vector<cv::Point> points;
        fitLine(result, points, 1, numSegments, 10, 200, 0);

        // Compute the angle of the lines for each segment into an array 
        std::vector<float> angles;
        for (size_t i = 1; i < points.size(); ++i) {
            float angle = calculateAngle(points[i - 1], points[i]);
            angles.push_back(angle);
        }
    }
}

void method2(cv::Mat& frame, cv::Mat& result, ROIData roiData) {
    if (roiData.roiSelected && roiData.pointCount == 4) {
        // Apply filtering steps using the helper function
        applyFilters(frame, roiData.points, result);

        // Convert result to grayscale for moment calculation
        cv::Mat grayResult;
        cv::cvtColor(result, grayResult, cv::COLOR_BGR2GRAY);

        // Split into a 4x4 grid and find moments for each segment
        int numSegmentsX = numSegments;  // Number of horizontal segments
        int numSegmentsY = numSegments;  // Number of vertical segments
        int segmentWidth = grayResult.cols / numSegmentsX;
        int segmentHeight = grayResult.rows / numSegmentsY;

        // Use a vector to store segment moments
        std::vector<std::vector<cv::Moments>> segmentMoments(numSegmentsY, std::vector<cv::Moments>(numSegmentsX));
       

        // Process each segment
        for (int y = 0; y < numSegmentsY; ++y) {
            for (int x = 0; x < numSegmentsX; ++x) {
                cv::Rect segmentRect(x * segmentWidth, y * segmentHeight, segmentWidth, segmentHeight);
                cv::Mat segment = grayResult(segmentRect);

                // Calculate moments for the segment
                segmentMoments[y][x] = cv::moments(segment, true);
            }
        }

        // Process moments and assign color to each pixel based on orientation
        for (int y = 0; y < numSegmentsY; ++y) {
            for (int x = 0; x < numSegmentsX; ++x) {
                double cx = segmentMoments[y][x].m10 / segmentMoments[y][x].m00 + x * segmentWidth;
                double cy = segmentMoments[y][x].m01 / segmentMoments[y][x].m00 + y * segmentHeight;

                // Calculate the orientation angle for the segment
                double angle = 0.0;
                if (segmentMoments[y][x].mu11 != 0.0) {
                    angle = 0.5 * atan2(2.0 * segmentMoments[y][x].mu11, (segmentMoments[y][x].mu20 - segmentMoments[y][x].mu02));
                }

                // Normalize the angle to [0, 360] degrees
                angle = angle * 180 / CV_PI;  // Convert to degrees
                if (angle < 0) {
                    angle += 360;  // Ensure angle is positive
                }

                // Map angle to a color in HSV space (H: 0-180, S: 255, V: 255)
                int hue = static_cast<int>(angle) * 180 / 360;  // Scale angle to [0, 179] for Hue
                cv::Scalar color(hue, 255, 255);  // Full saturation and value for bright colors

                // Convert from HSV to BGR for OpenCV visualization
                cv::Mat colorBGR;
                cv::cvtColor(cv::Mat(1, 1, CV_8UC3, color), colorBGR, cv::COLOR_HSV2BGR);

                // Now, color each pixel in the segment with the calculated color based on its orientation
                for (int dy = 0; dy < segmentHeight; ++dy) {
                    for (int dx = 0; dx < segmentWidth; ++dx) {
                        int px = x * segmentWidth + dx;
                        int py = y * segmentHeight + dy;
                        result.at<cv::Vec3b>(py, px) = colorBGR.at<cv::Vec3b>(0,0);  // Apply the color to the result
                    }
                }

                // Optionally, draw the centroid on the result image (as a red dot)
                //cv::circle(result, cv::Point(cvRound(cx), cvRound(cy)), 5, cv::Scalar(0, 0, 255), -1);

                // Optionally, display moments info for the segment
                std::cout << "Segment (" << y + 1 << ", " << x + 1 << ") Centroid: (" << cx << ", " << cy << ")" << std::endl;
                std::cout << "Segment (" << y + 1 << ", " << x + 1 << ") Orientation Angle: " << angle << " degrees" << std::endl;
            }
        }
        // mask off any red pixels
        // keep non red pixels





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
    cv::createTrackbar("Num Segments", "Original Video", &numSegments, 200);
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



cv::Point findBrightestPointInRegion(const cv::Mat& image, int dir, int regionStart, int regionEnd, int regionSize, int width, int brightnessThreshold) {
    double maxBrightness = 0;
    cv::Point brightestPoint(-1, -1);

    // Define the iteration direction based on the dir parameter
    int xStart = (dir == 1) ? 0 : width - regionSize + 1;
    int xEnd = (dir == 1) ? width - regionSize + 1 : -1;
    int xStep = (dir == 1) ? 1 : -1;

    // Iterate over the vertical region (y-direction)
    for (int y = regionStart; y < regionEnd - regionSize + 1; ++y) {
        // Iterate over the horizontal region (x-direction) based on dir
        for (int x = xStart; (dir == 1) ? x < xEnd : x > xEnd; x += xStep) {
            cv::Rect regionRect(x, y, regionSize, regionSize);
            cv::Mat region = image(regionRect);
            double avgBrightness = cv::mean(region)[0];

            if (avgBrightness > maxBrightness) {
                maxBrightness = avgBrightness;
                brightestPoint = cv::Point(x + regionSize / 2, y + regionSize / 2);
            }
        }
    }

    // Return the brightest point if it exceeds the brightness threshold
    return (maxBrightness > brightnessThreshold && brightestPoint.x != -1 && brightestPoint.y != -1) ? brightestPoint : cv::Point(-1, -1);
}




std::vector<cv::Point> processRegions(const cv::Mat& image, int dir, int numDivisions, int regionSize, int brightnessThreshold) {
    int rows = image.rows;
    int cols = image.cols;
    int divisionHeight = rows / numDivisions;
    std::vector<cv::Point> points;
    for (int i = 0; i < numDivisions; ++i) {
            int regionStart = i * divisionHeight;
            int regionEnd = std::min(regionStart + divisionHeight, rows);

            cv::Point brightestPoint = findBrightestPointInRegion(image, dir, regionStart, regionEnd, regionSize, cols, brightnessThreshold);

            if (brightestPoint.x != -1 && brightestPoint.y != -1) {
                points.push_back(brightestPoint);
            }
        }


    return points;
}


void connectPoints(cv::Mat& image, const std::vector<cv::Point>& points) {
    for (size_t i = 1; i < points.size(); ++i) {
        cv::line(image, points[i - 1], points[i], cv::Scalar(255, 0, 0), 2);  // Red lines between points
    }
}
void fitLine(cv::Mat& image, std::vector<cv::Point>& points,  int dir, int numDivisions, int regionSize, int brightnessThreshold, float angleDegrees) {
    int rows = image.rows;
    int cols = image.cols;
    int divisionHeight = rows / numDivisions;

    drawGridLines(image, numDivisions, divisionHeight);  // Draw grid lines
    
    // Process the regions to find the brightest points
    points = processRegions(image,  dir, numDivisions, regionSize, brightnessThreshold);

    // Draw the points on the image
    for (const cv::Point& point : points) {
        cv::circle(image, point, 2, cv::Scalar(0, 255, 0), -1);  // Swollen green point
    }

    // Vector to store the angles of each line segment
    std::vector<float> angles;

    // Draw the lines, angles, and store angles
    for (size_t i = 1; i < points.size(); ++i) {
        cv::line(image, points[i - 1], points[i], cv::Scalar(255, 0, 0), 2);  // Red lines for spline

        // Calculate the angle of the current line
        float angle = calculateAngle(points[i - 1], points[i]);

        // Store the angle in the array
        angles.push_back(angle);
    }

    // Print the angles as a table
    /*
    std::cout << "Line Segment | Angle (°)" << std::endl;
    std::cout << "------------------------" << std::endl;
    for (size_t i = 0; i < angles.size(); ++i) {
        std::cout << "Segment " << i + 1 << " | " << angles[i] << std::endl;
    }
    */
    // Connect the original points (optional, if you want to visualize the original points as well)
    connectPoints(image, points);
}

#endif // VISION_H