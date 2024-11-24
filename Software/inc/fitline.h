#ifndef FIT_LINE_H
#define FIT_LINE_H


#include <opencv2/opencv.hpp>




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
void fitLine(cv::Mat& image, int dir, int numDivisions, int regionSize, int brightnessThreshold, float angleDegrees) {
    int rows = image.rows;
    int cols = image.cols;
    int divisionHeight = rows / numDivisions;

    drawGridLines(image, numDivisions, divisionHeight);  // Draw grid lines
    
    // Process the regions to find the brightest points
    std::vector<cv::Point> points = processRegions(image,  dir, numDivisions, regionSize, brightnessThreshold);

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
    std::cout << "Line Segment | Angle (°)" << std::endl;
    std::cout << "------------------------" << std::endl;
    for (size_t i = 0; i < angles.size(); ++i) {
        std::cout << "Segment " << i + 1 << " | " << angles[i] << std::endl;
    }

    // Connect the original points (optional, if you want to visualize the original points as well)
    connectPoints(image, points);
}


#endif