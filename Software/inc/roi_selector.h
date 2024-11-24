#ifndef ROI_SELECTOR_H
#define ROI_SELECTOR_H

#include <opencv2/opencv.hpp>

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
#endif // ROI_SELECTOR_H
