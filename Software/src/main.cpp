#include <opencv2/opencv.hpp>
#include <iostream>

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



// Function to draw an "X" at the given point
void drawX(cv::Mat& image, cv::Point point) {
    int size = 5;  // Size of the "X"
    cv::line(image, point - cv::Point(size, size), point + cv::Point(size, size), cv::Scalar(0, 0, 255), 2);
    cv::line(image, point - cv::Point(size, -size), point + cv::Point(size, -size), cv::Scalar(0, 0, 255), 2);
}

void drawGridLines(cv::Mat& image, int numDivisions, int divisionHeight) {
    int cols = image.cols;
    for (int i = 1; i < numDivisions; ++i) {
        int y = i * divisionHeight;
        cv::line(image, cv::Point(0, y), cv::Point(cols, y), cv::Scalar(0, 255, 0), 2);  // Green grid lines
    }
}
cv::Point findBrightestPointInRegion(const cv::Mat& image, int regionStart, int regionEnd, int regionSize, int width, int brightnessThreshold) {
    double maxBrightness = 0;
    cv::Point brightestPoint(-1, -1);

    for (int y = regionStart; y < regionEnd - regionSize + 1; ++y) {
        for (int x = 0; x < width - regionSize + 1; ++x) {
            cv::Rect regionRect(x, y, regionSize, regionSize);
            cv::Mat region = image(regionRect);
            double avgBrightness = cv::mean(region)[0];

            if (avgBrightness > maxBrightness) {
                maxBrightness = avgBrightness;
                brightestPoint = cv::Point(x + regionSize / 2, y + regionSize / 2);
            }
        }
    }

    return (maxBrightness > brightnessThreshold && brightestPoint.x != -1 && brightestPoint.y != -1) ? brightestPoint : cv::Point(-1, -1);
}
std::vector<cv::Point2f> transformPointsToBirdsEyeView(const std::vector<cv::Point>& points, float angleDegrees) {
    std::vector<cv::Point2f> transformedPoints;
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(0, 0), angleDegrees, 1);  // Rotate by the given angle

    for (const auto& point : points) {
        cv::Mat pointMat = (cv::Mat_<double>(3, 1) << point.x, point.y, 1);  // Homogeneous coordinates
        cv::Mat transformedPointMat = rotationMatrix * pointMat;
        transformedPoints.push_back(cv::Point2f(transformedPointMat.at<double>(0), transformedPointMat.at<double>(1)));
    }

    return transformedPoints;
}
// Function to calculate the angle of a line between two points
float calculateAngle(cv::Point p1, cv::Point p2) {
    float angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180 / CV_PI;  // Convert from radians to degrees
    return angle;
}

std::vector<cv::Point> processRegions(const cv::Mat& image, int numDivisions, int regionSize, int brightnessThreshold) {
    int rows = image.rows;
    int cols = image.cols;
    int divisionHeight = rows / numDivisions;
    std::vector<cv::Point> points;

    for (int i = 0; i < numDivisions; ++i) {
        int regionStart = i * divisionHeight;
        int regionEnd = std::min(regionStart + divisionHeight, rows);

        cv::Point brightestPoint = findBrightestPointInRegion(image, regionStart, regionEnd, regionSize, cols, brightnessThreshold);

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
void fitLine(cv::Mat& image, int numDivisions, int regionSize, int brightnessThreshold, float angleDegrees) {
    int rows = image.rows;
    int cols = image.cols;
    int divisionHeight = rows / numDivisions;

    drawGridLines(image, numDivisions, divisionHeight);  // Draw grid lines

    // Process the regions to find the brightest points
    std::vector<cv::Point> points = processRegions(image, numDivisions, regionSize, brightnessThreshold);

    // Draw the points on the image
    for (const cv::Point& point : points) {
        cv::circle(image, point, 10, cv::Scalar(0, 255, 0), -1);  // Swollen green point
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


void printHelp() {
    std::cout << "Usage: VideoROI [options]\n"
              << "Options:\n"
              << "  --source [camera|udp]       Video source (default: camera).\n"
              << "  --udp_address [IP]          UDP stream address (default: 192.168.1.122).\n"
              << "  --udp_port [PORT]           UDP stream port (default: 5000).\n"
              << "  --help                      Display this help message.\n";
}
// Global variables for sliders
int threshold_value = 190;  // Default threshold value
int numSegments = 4;        // Default number of segments
int kernelSize = 5;         // Default kernel size

// Callback functions for sliders
void onThresholdSlider(int, void*) {
    std::cout << "Threshold: " << threshold_value << std::endl;
}

void onNumSegmentsSlider(int, void*) {
    std::cout << "Number of Segments: " << numSegments << std::endl;
}

void onKernelSizeSlider(int, void*) {
    std::cout << "Kernel Size: " << kernelSize << std::endl;
}

int main(int argc, char* argv[]) {
    // Open the video stream (camera or UDP source as before)
    cv::VideoCapture cap(1);  // Default to camera
    if (!cap.isOpened()) {
        std::cerr << "Error: Couldn't open the video stream or camera.\n";
        return -1;
    }

    // Set up ROI data and mouse callback
    ROIData roiData;
    cv::namedWindow("Original Video", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Original Video", selectROI, &roiData);

    // Create sliders
    cv::createTrackbar("Threshold", "Original Video", &threshold_value, 255, onThresholdSlider);
    cv::createTrackbar("Num Segments", "Original Video", &numSegments, 25, onNumSegmentsSlider);
    cv::createTrackbar("Kernel Size", "Original Video", &kernelSize, 70, onKernelSizeSlider);

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "Error: Couldn't read frame.\n";
            break;
        }

        // Draw the selected points and the trapezium on the frame
        if (roiData.pointCount > 0) {
            for (size_t i = 0; i < roiData.points.size(); ++i) {
                cv::circle(frame, roiData.points[i], 5, cv::Scalar(0, 0, 255), -1);  // Draw points in red
            }

            if (roiData.pointCount == 4) {
                // Draw the trapezium by connecting the 4 points
                cv::polylines(frame, roiData.points, true, cv::Scalar(0, 255, 0), 2);  // Draw the trapezium in green
            }
        }

        // Process ROI if selected
        if (roiData.roiSelected && roiData.pointCount == 4) {
            cv::Mat roiFrame;
            // Apply ROI to the frame (using the selected trapezium points)
            applyROIToFrameWithPadding(frame, roiData.points, roiFrame, 10);

            // Convert the cropped ROI to grayscale
            cv::Mat grayROI;
            cv::cvtColor(roiFrame, grayROI, cv::COLOR_BGR2GRAY);

            // Use sliders' values in the processing function
            fitLine(grayROI, numSegments, kernelSize, threshold_value, 10);

            // Display the gray ROI with marks
            cv::imshow("Gray ROI", grayROI);
        }

        // Display the original frame with selected points and trapezium
        cv::imshow("Original Video", frame);

        // Check for 'q' key press to exit
        int key = cv::waitKey(10);
        if (key == 'q' || key == 27) {  // 27 is the ASCII for 'Esc' key
            break;
        }
    }

    // Print ROI information if selected
    if (roiData.roiSelected) {
        std::cout << "Selected ROI points:\n";
        for (const auto& point : roiData.points) {
            std::cout << "(" << point.x << ", " << point.y << ")\n";
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}