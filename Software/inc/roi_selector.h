#ifndef ROI_SELECTOR_H
#define ROI_SELECTOR_H

#include <opencv2/opencv.hpp>

void selectROI(int event, int x, int y, int flags, void* param);

struct ROIData {
    cv::Rect roi;
    bool drawing = false;
    bool roiSelected = false;
};

#endif // ROI_SELECTOR_H
