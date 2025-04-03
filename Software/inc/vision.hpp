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

#include "../common/coms.hpp"

#include "../inc/calibration.hpp"

#include "../inc/peakTracker.hpp"


namespace viz {

const char* NODE_NAME = "VISION";

// -------------- Program Modes -------------- //
enum { M_INIT = 0, M_CALIB_BEV, M_CALIB_LENS, M_BANG_BANG, M_PATHFOLLOW, M_SEG_FLOOR, M_BIRD, M_VIZ, NUM_MODES };
// -------------- Parameter IDs -------------- //
enum { P_PROG_MODE = 0, P_LOOK_HRZ_HEIGHT, P_MAX_VEL, P_NAV_EN, P_KP, P_KI, P_KD, NUM_PARAMS }; 
// -------------- Global Variables -------------- //
std::atomic<bool> _exit_trig(false);
// -------------- Frame Setup -------------- //
const uint16_t WIDTH = 640;
const uint16_t HEIGHT = 480;

// ************************ Feature Detection ************************ //
struct WallDetection {
    uint16_t floor_y;   // Integer for the floor position (y-coordinate)
    float angle;  // Angle of the wall  wrt camera 
};

struct PathArc {
    float curvature;
    float radius;
    float angle;
    float length;
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
    WallDetection wall_detection = {0, 0.0f};
    if (!midpoints.empty()) {
        std::nth_element(midpoints.begin(), midpoints.begin() + midpoints.size() / 2, midpoints.end());
        float median_horizon = static_cast<float>(midpoints[midpoints.size() / 2]);
        median_horizon = std::clamp(median_horizon, 0.0f, static_cast<float>(y_plane.rows) / 5); // Clamp to valid range
        static float prev_horizon = median_horizon;
        
        float horz_lpf = iir_filter(median_horizon, prev_horizon, 0.2f);
        prev_horizon = horz_lpf; // Update the previous horizon value for the next iteration

        // Estimate wall angle using linear regression
        std::vector<cv::Point> edge_points;
        for (uint16_t y = 0; y < rows; y++) {
            for (uint16_t x = 0; x < cols; x++) {
                if (y_plane.at<uint8_t>(y, x) > 128) { // Threshold to find edges
                    edge_points.emplace_back(x, y);
                }
            }
        }
        static float current_angle = 0.0f; // Declare static variable outside the block
        if (!edge_points.empty()) {
            cv::Vec4f line;
            cv::fitLine(edge_points, line, cv::DIST_L2, 0, 0.01, 0.01);
            float angle = 0.0f;
            angle = std::atan2(line[1], line[0]) * 180.0f / M_PI; // Calculate angle in radians
            angle = std::clamp(angle, -5.0f, 5.0f); // Limit angle 
            float angle_lpf = iir_filter(angle, current_angle, 0.1f);  // Low-pass filter the angle
            current_angle = angle_lpf; // Update the static variable
        }

        wall_detection = {
            static_cast<uint16_t>(horz_lpf), current_angle
        };
    }

    return wall_detection;
}

void extractFloorPlane(const cv::Mat& y_plane, cv::Mat& floor_frame) {
    // Estimate the floor horizon and angle
    viz::WallDetection walls = viz::estimateWallHorizon(y_plane);
    int look_hrz_height = static_cast<int>(walls.floor_y);
    const cv::Mat roi_frame = y_plane(cv::Rect(0, look_hrz_height, viz::WIDTH, viz::HEIGHT - look_hrz_height));
    const cv::Mat rotation_matrix = cv::getRotationMatrix2D(
        cv::Point2f(viz::WIDTH / 2, viz::HEIGHT / 2), -walls.angle, 1.0);
    cv::Mat rotated_frame;
    cv::warpAffine(roi_frame, floor_frame, rotation_matrix, roi_frame.size(),
                   cv::INTER_AREA, cv::BORDER_CONSTANT, cv::Scalar(0));
}

// Find track estimate using homography transformation
void detectTrackEdges(const cv::Mat& roiFrame, cv::Mat& edges, const cv::Mat& homography) {
    // ------- Edge Detection ------- //
    //cv::GaussianBlur(roiFrame, edges, cv::Size(5, 21), 0);
    cv::Sobel(roiFrame, edges, CV_8U, 1, 0, 3, 2, 0, cv::BORDER_DEFAULT);
    cv::medianBlur(edges, edges, 5);
    //  mask warped img boundaries edges fromm sobel
    std::vector<cv::Point> contour = {
        cv::Point(10, 0),
        cv::Point(roiFrame.cols-10, 0),
        cv::Point(roiFrame.cols/2 +200, roiFrame.rows),
        cv::Point(roiFrame.cols/2 -200, roiFrame.rows)
    };

    cv::line(edges, contour[0], contour[3], cv::Scalar(0), 30);
    cv::line(edges, contour[1], contour[2], cv::Scalar(0), 30); // Mask warped img boundaries
}

class PathEstimator {
public:
    PathEstimator() :
        topTracker(640), midTracker(640), lowTracker(640),
        top_pathTrend{0}, mid_pathTrend{0}, low_pathTrend{0},
        topRoi(cv::Rect(0, 0, 640, 480 / 5)),
        midRoi(cv::Rect(0, 480 * 2 / 5, 640, 480 / 5)),
        lowRoi(cv::Rect(0, 480 * 4 / 5, 640, 480 / 5)) {
        }
    void findPathWaypoints(const cv::Mat& trackFrame){
        // -- Waypoint Detection -
        pkt::Peak topPeak = topTracker.topologicalPeakTrack(trackFrame, topRoi, top_pathTrend);
        pkt::Peak midPeak = midTracker.topologicalPeakTrack(trackFrame, midRoi, mid_pathTrend);
        pkt::Peak lowPeak = lowTracker.topologicalPeakTrack(trackFrame, lowRoi, low_pathTrend);
    }

    void findPathArc(){
        // -- Path Arc Estimation --

    }

    inline const PathArc& getPathArc() const {return pathArc;}

    private:
        pkt::PeakTracker topTracker;
        pkt::PeakTracker midTracker;
        pkt::PeakTracker lowTracker;
        std::array<float, WIDTH> top_pathTrend;
        std::array<float, WIDTH> mid_pathTrend;
        std::array<float, WIDTH> low_pathTrend;
        const cv::Rect topRoi;
        const cv::Rect midRoi;
        const cv::Rect lowRoi;
        PathArc pathArc;
};


// ******************************* Visualization ******************************* //
void drawTopology(cv::Mat& subFrame, const std::array<float, WIDTH>& hist, int peakIdx, int peakWidth, float peakProminence, cv::Scalar histColor, cv::Scalar peakColor) {
    //subFrame.setTo(0);
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

} // namespace viz

#endif // VISION_HPP
