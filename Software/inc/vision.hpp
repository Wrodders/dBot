#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <queue>
#include <condition_variable>   
#include <mutex>
#include <algorithm>
#include <atomic>
#include <array>
#include <vector>
#include <tuple>
#include <zmq.hpp>
#include <syslog.h>
#include <algorithm> 
#include <numeric>
#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include "../common/coms.hpp"

#include "../inc/calibration.hpp"


namespace viz {

const char* NODE_NAME = "VISION";

enum { M_INIT = 0, M_CALIB_BEV, M_CALIB_LENS, M_PATHFOLLOW, M_SEG_FLOOR, M_BIRD, M_VIZ, NUM_MODES };

// -------------- Parameter IDs -------------- //
enum { P_PROG_MODE = 0, P_LOOK_HRZ_HEIGHT, P_MAX_VEL, P_NAV_EN, NUM_PARAMS }; 

// -------------- Global Variables -------------- //
std::atomic<bool> _exit_trig(false);

// -------------- Frame Setup -------------- //

const uint16_t WIDTH = 640;
const uint16_t HEIGHT = 480;
const uint32_t FRAME_SIZE = WIDTH * HEIGHT * 3 / 2;

const std::vector<cv::Point2f> _src_pts = {
    cv::Point2f(0, HEIGHT), cv::Point2f(WIDTH, HEIGHT),
    cv::Point2f(0, 0), cv::Point2f(WIDTH, 0)
};    

std::array<cv::Point2f, 4> _dst_pts = { // homography transform destination points
    cv::Point2f(WIDTH / 2 - 90, HEIGHT - 360),
    cv::Point2f(WIDTH / 2 + 90, HEIGHT - 360),
    cv::Point2f(0, 0), 
    cv::Point2f(WIDTH, 0)
};

struct WallDetection {
    uint16_t floor_y;   // Integer for the floor position (y-coordinate)
};


//@brief: Estimate the horizon height of the floor
WallDetection estimateWallHorizon(const cv::Mat& y_plane) {
    cv::Mat integral_img;
    cv::integral(y_plane, integral_img, CV_32S);
    uint16_t strip_height = 5;
    uint16_t rows = y_plane.rows;
    uint16_t cols = y_plane.cols;
    std::vector<uint16_t> midpoints;
    for (uint16_t y = strip_height; y < rows - strip_height; y++) {
        int sum_above = integral_img.at<int>(y, cols - 1) - integral_img.at<int>(y - strip_height, cols - 1);
        int sum_below = integral_img.at<int>(y + strip_height, cols - 1) - integral_img.at<int>(y, cols - 1);
        if (std::abs(sum_above - sum_below) > 8000) { // large difference in integral sum
            midpoints.push_back(y);
        }
    }
    WallDetection wall_detection = {0};
    if(!midpoints.empty()) {
        std::nth_element(midpoints.begin(), midpoints.begin() + midpoints.size() / 2, midpoints.end());
        float median_horizon = static_cast<float>(midpoints[midpoints.size() / 2]);
        static float current_horizon = median_horizon;
        float horz_lpf = iir_filter(median_horizon, current_horizon, 0.2f);
        wall_detection = {
            static_cast<uint16_t>(horz_lpf)
        };
    }

    return wall_detection;
}



// Find track estimate using homography transformation
void detectTrackEdges(cv::Mat& roiFrame, const cv::Mat& homography) {
    cv::Mat warped = cv::Mat::zeros(roiFrame.size(), CV_8U);
    //cv::warpPerspective(roiFrame, warped, homography, warped.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    // ------- Edge Detection ------- //
    // Apply Gaussian Blur to reduce noise
    cv::GaussianBlur(roiFrame, roiFrame, cv::Size(21, 5), 0);
    // Apply Sobel operator to detect edges
    cv::Sobel(roiFrame, roiFrame, CV_8U, 1, 0, 3, 2, 0, cv::BORDER_DEFAULT);
    cv::threshold(roiFrame, roiFrame, 80, 255, cv::THRESH_BINARY);

}

// Computes the histogram from a ROI of the edge image,
// normalizes it to 0-255 and applies Gaussian blur to reduce high-frequency noise.
void computeLineTrend(const cv::Mat& edges, const cv::Rect& roi, const std::array<float, WIDTH>& hist) {
    cv::Mat section = edges(roi);
    // Sum vertically to collapse the ROI into a single row histogram.
    cv::reduce(section, hist, 0, cv::REDUCE_SUM, CV_32F);
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);  // Normalize to 0-255 range
    cv::GaussianBlur(hist, hist, cv::Size(21, 1), 0, 0);   // Reduce positional high-frequency noise
}

// Identify candidate peaks in the histogram given a noise floor.
// Each candidate is a tuple: (peak index, width, prominence)
std::vector<std::tuple<int, int, float>> identifyPeakCandidates(const std::array<float, WIDTH>& hist, float noiseFloor) {
    std::vector<std::tuple<int, int, float>> candidates;

    for (size_t i = 1; i + 1 < hist.size(); ++i) {
        float val = hist[i];
        // Check if current point is a local peak and above the noise floor.
        if (val > hist[i - 1] &&
            val > hist[i + 1] &&
            val > noiseFloor) {

            int leftValley = i;
            int rightValley = i;
            // Search to the left for the valley.
            while (leftValley > 0 && hist[leftValley] > noiseFloor) {
                leftValley--;
            }
            int lengthHist = hist.size();
            // Search to the right for the valley.
            while (rightValley + 1 < lengthHist && hist[rightValley] > noiseFloor) {
                rightValley++;
            }
            int width = rightValley - leftValley;

            // Determine valley values (handling edge cases).
            float leftValleyVal = (leftValley > 0) ? hist[leftValley] : 0.0f;
            float rightValleyVal = (rightValley < lengthHist - 1) ? hist[rightValley] : 0.0f;
            float prominence = val - std::max(leftValleyVal, rightValleyVal);

            // If a peak extends to an edge, use overall maximum as reference.
            if (leftValley == 0 || rightValley == lengthHist - 1) {
                float maxVal = *std::max_element(hist.begin(), hist.end());
                prominence = val - maxVal;
            }

            candidates.emplace_back(i, width, prominence);
        }
    }
    return candidates;
}

// Analyze candidate peaks to compute the mean prominence and dynamic range.
std::tuple<float, float> analyzePeaks(const std::vector<std::tuple<int, int, float>>& candidates) {
    if (candidates.empty())
        return {0.0f, 0.0f};

    std::vector<float> prominences;
    for (const auto& candidate : candidates) {
        prominences.push_back(std::get<2>(candidate)); // Extract prominence
    }
    float meanProminence = std::accumulate(prominences.begin(), prominences.end(), 0.0f) / prominences.size();
    float maxProminence = *std::max_element(prominences.begin(), prominences.end());
    return {meanProminence, maxProminence - meanProminence};
}

// Find the dominant peak from candidates starting the search near the last known peak index.
// If the candidate peak is significantly lower than the overall maximum in the histogram,
// it indicates that the track has significantly changed, so the algorithm can fall back to a wider search.
std::tuple<int, int, float> findDominantPeak(
    const std::array<float, WIDTH>& hist,
    const std::vector<std::tuple<int, int, float>>& candidates,
    float meanProminence, int lastPeakIndex)
{
    if (candidates.empty())
        return {0, 0, 0.0f};

    // Group peaks that are near each other (within 10 indices).
    std::vector<std::vector<std::tuple<int, int, float>>> peakGroups;
    std::vector<std::tuple<int, int, float>> currentGroup;
    for (const auto& candidate : candidates) {
        int peakIndex = std::get<0>(candidate);
        if (currentGroup.empty() || (peakIndex - std::get<0>(currentGroup.back()) < 10)) {
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
    std::vector<std::tuple<int, int, float>> filteredCandidates;
    for (const auto& group : peakGroups) {
        for (const auto& candidate : group) {
            if (std::get<2>(candidate) > meanProminence)
                filteredCandidates.push_back(candidate);
        }
    }
    // If none remain after filtering, use all candidates.
    if (filteredCandidates.empty()) {
        filteredCandidates = candidates;
    }

    // Select the candidate closest to the last known peak index.
    int bestIndex = std::get<0>(filteredCandidates[0]);
    float bestDistance = std::abs(lastPeakIndex - bestIndex);
    for (const auto& candidate : filteredCandidates) {
        int peakIndex = std::get<0>(candidate);
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
        if (std::get<0>(candidate) == bestIndex) {
            bestWidth = std::get<1>(candidate);
            bestProminence = std::get<2>(candidate);
            break;
        }
    }

    // If the selected peak's value is notably less than the maximum in the histogram,
    // assume a significant track change and set bestProminence to 0 to trigger fallback.
    double minVal, histMax;
    cv::minMaxLoc(hist, &minVal, &histMax);
    float candidateVal = hist[bestIndex];
    if (candidateVal < histMax) {
        // For example, if the candidate is less than 90% of the max, trigger fallback.
        if (candidateVal < 0.6 * histMax) {
            return {0, 0, 0.0f}; // Indicate that fallback should be used.
        }
    }

    return {bestIndex, bestWidth, bestProminence};
}

// Fallback: perform a full scan using a lower threshold if the primary search fails.
std::tuple<int, int, float> fallbackFullScan(const std::array<float, WIDTH>& hist, float meanProminence, int lastPeakIndex) {
    auto candidates = identifyPeakCandidates(hist, 0.5f * meanProminence);
    return findDominantPeak(hist, candidates, meanProminence, lastPeakIndex);
}
// --- PeakTracker Class ---
class PeakTracker {
public:
    // Initializes the tracker with the histogram width and starting position.
    PeakTracker(int histCols)
        : lastPeakIndex(histCols / 2),
          prevFilteredRange(0.0f),
          prevFilteredWidth(0.0f),
          prevFilteredIndex(static_cast<float>(histCols / 2))
    {}

    // Performs topological peak tracking on a given ROI of the edge image.
    // Returns a tuple: (peak index, peak width, filtered dynamic range, histogram).
    std::tuple<uint16_t, uint16_t, uint8_t, std::array<float, WIDTH>>
    topologicalPeakTrack(const cv::Mat& edges, const cv::Rect& roi) {
        std::array<float, WIDTH> hist;
        computeLineTrend(edges, roi, hist);

        // Compute the noise floor as the median of the histogram values.
        std::array<float, WIDTH> histSorted = hist;
        std::sort(histSorted.begin(), histSorted.end());
        float medianNoiseFloor = histSorted[histSorted.size() / 2];
        // Identify candidate peaks.
        auto candidates = identifyPeakCandidates(hist, medianNoiseFloor);
        auto [meanProminence, peakDynamicRange] = analyzePeaks(candidates);

        // First attempt: search near the last known peak.
        auto [bestIndex, bestWidth, bestDynamicRange] = findDominantPeak(hist, candidates, meanProminence, lastPeakIndex);

        // If no valid dominant peak was found (signaled by bestDynamicRange==0),
        // or if the candidate's value is notably less than the histogram maximum,
        // fall back to a full scan.
        if (bestDynamicRange == 0) {
            std::tie(bestIndex, bestWidth, bestDynamicRange) = fallbackFullScan(hist, meanProminence, lastPeakIndex);
        }

        // Update the last known peak index.
        lastPeakIndex = bestIndex;

        // --- Smoothing using IIR filters ---
        prevFilteredRange = iir_filter(bestDynamicRange, prevFilteredRange, 0.6f);
        uint16_t filteredRange = static_cast<uint16_t>(prevFilteredRange);

        prevFilteredWidth = iir_filter(static_cast<float>(bestWidth), prevFilteredWidth, 0.5f);
        bestWidth = static_cast<int>(prevFilteredWidth);

        prevFilteredIndex = iir_filter(static_cast<float>(bestIndex), prevFilteredIndex, 0.9f);
        bestIndex = static_cast<int>(prevFilteredIndex);

        return {static_cast<uint16_t>(bestIndex),
                static_cast<uint16_t>(bestWidth),
                static_cast<uint8_t>(filteredRange),
                hist};
    }

private:
    int lastPeakIndex;
    float prevFilteredRange;
    float prevFilteredWidth;
    float prevFilteredIndex;
};

// Draw topology (visualization) on the frame
void drawTopology(cv::Mat& subFrame, const std::array<float, WIDTH>& hist, int peakIdx, int peakWidth, float peakProminence, cv::Scalar histColor, cv::Scalar peakColor) {
    subFrame.setTo(0);
    // ------- Visualize Dominant Peak ------- //

    // ------- Visualize Histogram ------- //
    for (size_t i = 0; i < hist.size(); ++i) {
        int scaledHeight = static_cast<int>(hist[i] * subFrame.rows / 255.0f);
        cv::line(subFrame, cv::Point(i, subFrame.rows), cv::Point(i, subFrame.rows - scaledHeight), histColor);    }

    if (peakIdx >= 0) {
        int scaledProminence = static_cast<int>(peakProminence * subFrame.rows / 255.0f);
        cv::Rect peakRect(peakIdx - peakWidth / 2, subFrame.rows - scaledProminence, peakWidth, scaledProminence);
        cv::rectangle(subFrame, peakRect, peakColor, -1);
    }
}

void displayStats(cv::Mat& subFrame, float peakProminence, int peakWidth, float drivability) {
    cv::putText(subFrame, "Dominant Prominence: " + std::to_string(peakProminence),
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(subFrame, "Dominant Width: " + std::to_string(peakWidth),
                cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(subFrame, "Drivability: " + std::to_string(drivability),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
}


struct TrackSection {
    int peakIdx;
    int peakWidth;
    float peakProminence;
};

void path_follower(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map) {
    // -- Floor Segmentation --
    viz::WallDetection walls = {0};
    walls = viz::estimateWallHorizon(undistorted);
    int look_hrz_height = static_cast<int>(walls.floor_y);
    cv::Rect floor_roi;  // Dynamically adjust the ROI based on the floor horizon 
    cv::Mat floor_frame = undistorted(cv::Rect(0,look_hrz_height, WIDTH, HEIGHT - look_hrz_height));
    viz::detectTrackEdges(floor_frame, homography_matrix);
    // -- Peak Tracking --
    viz::PeakTracker lowTracker(floor_frame.cols);
    std::array<float, WIDTH> low_pathTrend;
    struct TrackSection low_section;
    const  cv::Rect lowRoi(0, floor_frame.rows * 3 / 4, viz::WIDTH, floor_frame.rows / 4);
    std::tie(low_section.peakIdx, low_section.peakWidth, low_section.peakProminence, low_pathTrend) =
        lowTracker.topologicalPeakTrack(floor_frame, lowRoi);

    viz::PeakTracker highTracker(floor_frame.cols);
    struct TrackSection high_section;
    std::array<float, WIDTH> high_pathTrend;
    const cv::Rect highRoi(0, 0, viz::WIDTH, floor_frame.rows / 4);
    std::tie(high_section.peakIdx, high_section.peakWidth, high_section.peakProminence, high_pathTrend) =
        highTracker.topologicalPeakTrack(floor_frame, highRoi);
    // -- Visualization --

    cv::Mat trackVizFrame = outputFrame(floor_roi);
    cv::Mat topVizFrame = outputFrame(cv::Rect(0, 0, viz::WIDTH, walls.floor_y / 2));
    cv::Mat lowVizFrame = outputFrame(cv::Rect(0, walls.floor_y / 2, viz::WIDTH, walls.floor_y / 2));
    viz::drawTopology(topVizFrame, high_pathTrend, high_section.peakIdx, high_section.peakWidth, high_section.peakProminence, cv::Scalar(128), cv::Scalar(255));
    viz::drawTopology(lowVizFrame, low_pathTrend, low_section.peakIdx, low_section.peakWidth, low_section.peakProminence, cv::Scalar(64), cv::Scalar(192));
    floor_frame.copyTo(trackVizFrame);
}



void img_pipeline(const cv::Mat& undistorted, cv::Mat& outputFrame, cv::Mat& homography_matrix, ParameterMap& param_map, int& loopRate_ms) {
    float progmode;
    param_map.get_value(viz::P_PROG_MODE, progmode);
    static std::vector<cv::Point2f> imagePts(4);
    switch ((int)progmode) {
        case viz::M_INIT:
            syslog(LOG_INFO, "Initializing Vision Pipeline");
            param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
            break;
        case viz::M_CALIB_BEV: {
            static auto start_time = std::chrono::steady_clock::now();
            static bool timeout_active = false; // inital 
            // If re-entering calibration mode, reset the timer
            if (!timeout_active) {
                start_time = std::chrono::steady_clock::now();
                timeout_active = true;
            }
            int ret = calib::calibrateBirdseye(undistorted, homography_matrix, imagePts);
            undistorted.copyTo(outputFrame); // Show the chessboard for debugging        
            if (ret == 1) {
                syslog(LOG_INFO, "Calibration Complete");
                param_map.set_value(viz::P_PROG_MODE, viz::M_BIRD);
                loopRate_ms = -1;
                timeout_active = false;  // Reset timeout for next calibration
            } else if (ret == -1) {
                syslog(LOG_ERR, "Calibration Failed");
                param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
                loopRate_ms = -1;
                timeout_active = false;  // Reset timeout for next calibration
            } else {
                // Check if timeout has occurred
                if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(60)) {
                    syslog(LOG_ERR, "Calibration Timeout");
                    param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
                    loopRate_ms = -1;
                    timeout_active = false;  // Reset timeout for next calibration
                } else {
                    cv::putText(outputFrame, "Chessboard not found", cv::Point(30, 30),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(64), 2);
                    loopRate_ms = 500;
                }
            }
            }
            break;
        case viz::M_CALIB_LENS: {
            static auto start_time = std::chrono::steady_clock::now();
            static bool timeout_active = false; // inital 
            // If re-entering calibration mode, reset the timer
            if (!timeout_active) {
                start_time = std::chrono::steady_clock::now();
                timeout_active = true;
            }
        }
            break;
        
        case viz::M_PATHFOLLOW:
            path_follower(undistorted, outputFrame, homography_matrix, param_map);
            break;
        case viz::M_SEG_FLOOR:{
            struct viz::WallDetection walls = {0};
            walls = viz::estimateWallHorizon(undistorted);
            undistorted.copyTo(outputFrame);
            cv::line(outputFrame, cv::Point(0, walls.floor_y), cv::Point(viz::WIDTH, walls.floor_y), cv::Scalar(128), 30);
        }
            break;
        case M_BIRD:{
            // Apply birds eye view perspective transformation
            float horz_height;
            param_map.get_value(viz::P_LOOK_HRZ_HEIGHT, horz_height);
            if(homography_matrix.empty()) {
                //syslog(LOG_ERR, "Homography matrix is empty");
                param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ); // Fall back to pass through mode
                break;
            }
            homography_matrix.at<double>(2, 2) = horz_height;
            cv::warpPerspective(undistorted, outputFrame, homography_matrix, undistorted.size(), cv::INTER_AREA);
            undistorted.copyTo(outputFrame);
            cv::circle(outputFrame, imagePts[0], 5, cv::Scalar(64), -1); 
            cv::circle(outputFrame, imagePts[1], 5, cv::Scalar(128), -1); 
            cv::circle(outputFrame, imagePts[2], 5, cv::Scalar(192), -1); 
            cv::circle(outputFrame, imagePts[3], 5, cv::Scalar(255), -1);       
        }
        break;
        case viz::M_VIZ: // Pass through mode
            undistorted.copyTo(outputFrame);
            break;
        default: 
            syslog(LOG_ERR, "Invalid Program Mode");
            param_map.set_value(viz::P_PROG_MODE, viz::M_VIZ);
            break;
    }
}

} // namespace viz

#endif // VISION_HPP
