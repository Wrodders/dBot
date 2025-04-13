#ifndef PEAKTRACKER_HPP
#define PEAKTRACKER_HPP

#include <opencv2/opencv.hpp>
#include "../common/coms.hpp"
#include <algorithm> 
#include <numeric>
#include <random>

namespace pk{

// ****************************** Peak Tracking ****************************** //
struct LineEstimate{
    float median;
    float max;
    int index;
    float prominence;
};
struct LineEstimate computeLineTrend(const cv::Mat& frame) {
    LineEstimate lineTrend;
    cv::Mat hist(1, frame.rows, CV_32F, cv::Scalar(0));
    cv::reduce(frame, hist, 0, cv::REDUCE_SUM, CV_32F);
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);    
    // Find the maximum value and its index.
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(hist, &minVal, &maxVal, &minLoc, &maxLoc);
    lineTrend.index = maxLoc.x;
    lineTrend.max = static_cast<float>(maxVal);
    float* histData = hist.ptr<float>(0);
    std::nth_element(histData, histData + frame.rows / 2, histData + frame.rows);
    lineTrend.median = histData[frame.rows / 2];
    // Find the prominence of the peak
    lineTrend.prominence = maxVal - lineTrend.median;
    return lineTrend;
}

}
#endif  // PEAKTRACKER_HPP