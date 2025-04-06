#ifndef PEAKTRACKER_HPP
#define PEAKTRACKER_HPP

#include <opencv2/opencv.hpp>
#include "../common/coms.hpp"
#include <algorithm> 
#include <numeric>

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

//@Brief: Finds Peaks Above the Noise Floor
//@Description: Searches around Local Peaks in the histogram to find valleys
std::vector<Peak> identifyPeakCandidates(const std::array<float, 640>& hist, float noiseFloor) {
    std::vector<Peak> candidates;
    int lengthHist = hist.size();
    for (int i = 1; i + 1 < lengthHist; ++i) {
        float val = hist[i];
        // Check if current point is a local peak and above the noise floor.
        if (val > hist[i - 1] && val > hist[i + 1] && val > noiseFloor) {
            int leftValleyIdx = i;
            int rightValleyIdx = i;
            // Expand to find the left valley
            while (leftValleyIdx > 0 && hist[leftValleyIdx] > noiseFloor) {
                leftValleyIdx--;
            }
            // Expand to find the right valley
            while (rightValleyIdx + 1 < lengthHist && hist[rightValleyIdx] > noiseFloor) {
                rightValleyIdx++;
            }
            int width = std::abs(rightValleyIdx - leftValleyIdx);
            
            // Determine valley values (handling edge cases)
            float leftValleyVal = (leftValleyIdx > 0) ? hist[leftValleyIdx] : 0.0f;
            float rightValleyVal = (rightValleyIdx < lengthHist - 1) ? hist[rightValleyIdx] : 0.0f;
            float prominence = val - std::max(leftValleyVal, rightValleyVal); // Topographic prominence to the left and right valleys
            Peak peak;
            peak.index = i;
            peak.value = val;
            peak.width = width;
            peak.leftValleyIdx = leftValleyIdx;
            peak.rightValleyIdx = rightValleyIdx;
            peak.prominence = prominence;
            candidates.push_back(peak);
        }
    }
    return candidates;
}

// Analyze candidate peaks to compute the mean prominence and dynamic range.
std::tuple<float, float> analyzePeaks(const std::vector<Peak>& candidates) {
    if (candidates.empty())
        return {0.0f, 0.0f};

    std::vector<float> prominences;
    for (const auto& candidate : candidates) {
        prominences.push_back(candidate.prominence); // Extract prominence
    }
    float meanProminence = std::accumulate(prominences.begin(), prominences.end(), 0.0f) / prominences.size();
    float maxProminence = *std::max_element(prominences.begin(), prominences.end());
    return {meanProminence, maxProminence - meanProminence};
}

// Find the dominant peak from candidates starting the search near the last known peak index.
// If the candidate peak is significantly lower than the overall maximum in the histogram,
// it indicates that the track has significantly changed, so the algorithm can fall back to a wider search.
struct Peak findDominantPeak(
    const std::array<float, 640>& hist,
    const std::vector<Peak>& candidates,
    float meanProminence, int lastPeakIndex)
{
    if (candidates.empty()) {return Peak{};}
    // Group peaks that are near each other (within 10 indices).
    std::vector<std::vector<Peak>> peakGroups;
    std::vector<Peak> currentGroup;
    for (const auto& candidate : candidates) {
        int peakIndex = candidate.index;
        if (currentGroup.empty() || (peakIndex - currentGroup.back().index < 10)) {
            currentGroup.push_back(candidate);
        } else {
            peakGroups.push_back(currentGroup);
            currentGroup.clear();
            currentGroup.push_back(candidate);
        }
    }
    if (!currentGroup.empty())
        peakGroups.push_back(currentGroup);

    // Filter out candidates with prominence below the mean.
    std::vector<Peak> filteredCandidates;
    for (const auto& group : peakGroups) {
        for (const auto& candidate : group) {
            if (candidate.prominence >= meanProminence) {
                filteredCandidates.push_back(candidate);
            }
        }
    }
    if (filteredCandidates.empty()) {
        return Peak{};
    }
    // Select the candidate closest to the last known peak index.
    int bestIndex = filteredCandidates[0].index;
    float bestDistance = std::abs(lastPeakIndex - bestIndex);
    for (const auto& candidate : filteredCandidates) {
        int peakIndex = candidate.index;
        float distance = std::abs(lastPeakIndex - peakIndex);
        if (distance < bestDistance) {
            bestIndex = peakIndex;
            bestDistance = distance;
        }
    }
    int bestWidth = 0;
    float bestProminence = 0.0f;
    // Retrieve width and prominence for the chosen candidate.
    for (const auto& candidate : filteredCandidates) {
        if (candidate.index == bestIndex) {
            bestWidth = candidate.width;
            bestProminence = candidate.prominence;
            break;
        }
    }
    // If the selected peak's value is notably less than the maximum in the histogram,
    // assume a significant track change this is potentially a false peak.
    double minVal, histMax;
    cv::minMaxLoc(hist, &minVal, &histMax);
    float candidateVal = hist[bestIndex];
    if (candidateVal < histMax) {
        if (candidateVal < 0.6 * histMax) {
            return Peak{};
        }
    }
    Peak bestPeak;
    bestPeak.index = static_cast<int>(bestIndex);
    bestPeak.value = static_cast<float>(candidateVal);
    bestPeak.width = static_cast<int>(bestWidth);
    bestPeak.leftValleyIdx = static_cast<int>(filteredCandidates[0].leftValleyIdx);
    bestPeak.rightValleyIdx = static_cast<int>(filteredCandidates[0].rightValleyIdx);
    bestPeak.prominence = static_cast<float>(bestProminence);
    return bestPeak;
}

// Fallback: perform a full scan using a lower threshold if the primary search fails.
struct Peak fallbackFullScan(const std::array<float, 640>& hist, float meanProminence, int lastPeakIndex) {
    auto candidates = identifyPeakCandidates(hist, 0.5f * meanProminence); // lower the threshold
    return findDominantPeak(hist, candidates, meanProminence, lastPeakIndex);
}

class PeakTracker {
public:
    // Initializes the tracker with the histogram width and starting position.
    PeakTracker() : hist_trend{} {
        std::fill(hist_trend.begin(), hist_trend.end(), 0.0f);
    }
    Peak topologicalPeakTrack(const cv::Mat& frame, const Peak& lastPeak) {
        struct LineTrend line_trend = computeLineTrend(frame, hist_trend);
        float medianNoiseFloor = line_trend.median;
        std::vector<Peak> candidates = identifyPeakCandidates(hist_trend, medianNoiseFloor*8);
        auto [meanProminence, peakDynamicRange] = analyzePeaks(candidates);
        // First attempt: search near the last known peak.
        struct Peak peak = findDominantPeak(hist_trend, candidates, meanProminence, lastPeak.index);
        if (peak.prominence == 0) {
            peak = fallbackFullScan(hist_trend, meanProminence, lastPeak.index);
        }
        return peak;
    }

private:
    std::array<float, 640> hist_trend;
};
}



#endif  // PEAKTRACKER_HPP