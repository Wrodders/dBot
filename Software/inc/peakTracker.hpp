#ifndef PEAKTRACKER_HPP
#define PEAKTRACKER_HPP

#include <opencv2/opencv.hpp>
#include "../common/coms.hpp"
#include <algorithm> 
#include <numeric>
#include <random>

namespace pk{

    // ****************************** Peak Tracking ****************************** //
struct Peak{
    int index; 
    float value; 
    int width;
    int leftValleyIdx;
    int rightValleyIdx;
    float prominence;

    Peak() : index(320), value(0.0f), width(0), leftValleyIdx(0), rightValleyIdx(0), prominence(0.0f) {}
};

// Computes the histogram from a ROI of the edge image,
// normalizes it to 0-255 and applies Gaussian blur to reduce high-frequency noise.
struct LineTrend{
    float median;
    float max;
    int index;
};
struct LineTrend computeLineTrend(const cv::Mat& frame, std::array<float, 640>& trend) {
    struct LineTrend lineTrend;
    cv::Mat hist(640, 1, CV_32F, cv::Scalar(0));
    cv::reduce(frame, hist, 0, cv::REDUCE_SUM, CV_32F);
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
    cv::Point est;
    cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &est);
    std::array<float, 640> histSorted = hist;
    std::sort(histSorted.begin(), histSorted.end());
    lineTrend.median = histSorted[histSorted.size() / 2];
    lineTrend.max = histSorted.back();
    lineTrend.index = est.x;
    for (int i = 0; i < hist.cols; ++i) {
        trend[i] = hist.at<float>(i);
    }
    return lineTrend;

}

class PeakTracker {
public:
    // Initializes the tracker with the histogram width and starting position.
    PeakTracker() : hist_trend{} {
        std::fill(hist_trend.begin(), hist_trend.end(), 0.0f);
    }
    Peak topologicalPeakTrack(const cv::Mat& frame) {
        struct LineTrend line_trend = computeLineTrend(frame, hist_trend);
        float medianNoiseFloor = line_trend.median;
        cv::GaussianBlur(hist_trend, hist_trend, cv::Size(1, 21), 0);
        Peak peak;
        peak.index = line_trend.index;
        peak.value = hist_trend[peak.index];
        peak.width = 10;
        peak.leftValleyIdx = line_trend.index - 10;
        peak.rightValleyIdx = line_trend.index + 10;
        peak.prominence = peak.value - medianNoiseFloor;
        return peak;
    }

    // Returns the histogram trend for visualization.
    const std::array<float, 640> getHistogram() const {
        return hist_trend;
    }

private:
    std::array<float, 640> hist_trend;
};
}



#endif  // PEAKTRACKER_HPP