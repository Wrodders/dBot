#include "../inc/roi_selector.h"

void selectROI(int event, int x, int y, int flags, void* param) {
    auto* data = static_cast<ROIData*>(param);

    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
            data->drawing = true;
            data->roi = {x, y, 0, 0}; // Initialize ROI
            break;
        case cv::EVENT_MOUSEMOVE:
            if (data->drawing) {
                data->roi.width = x - data->roi.x;
                data->roi.height = y - data->roi.y;
            }
            break;
        case cv::EVENT_LBUTTONUP:
            data->drawing = false;
            data->roiSelected = true;
            data->roi.width = x - data->roi.x;
            data->roi.height = y - data->roi.y;
            break;
        default:
            break;
    }
}
