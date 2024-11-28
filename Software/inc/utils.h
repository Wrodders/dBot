#ifndef CAM_UTILS_H
#define CAM_UTILS_H

#include <opencv2/opencv.hpp>

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

void removeEdgeAreasFromROI(cv::Mat& roiImage, const std::vector<cv::Point>& roiPoints) {
    // Create a mask with the same size as the ROI image
    cv::Mat mask = cv::Mat::zeros(roiImage.size(), CV_8UC1);  // Same size as the ROI image, black by default

    // Fill the polygon (ROI boundary) with white on the mask (representing the region)
    std::vector<std::vector<cv::Point>> contours = {roiPoints};
    cv::fillPoly(mask, contours, cv::Scalar(255));

    // Set the areas along the edge (the polygon) to black

    roiImage.setTo(cv::Scalar(255), mask);  // Black out the areas defined by the mask
}


#endif // UITILS_H