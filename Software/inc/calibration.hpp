#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP
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
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <syslog.h>
#include <vector>
#include <string>
#include <fstream>


#include <opencv2/core/eigen.hpp>

#include "../common/coms.hpp"


namespace calib {
    const std::string INTRINSICS_FILE = "calibration/Intrinsics.xml";
    const std::string DISTORTION_FILE = "calibration/Distortion.xml";
    const std::string HOMOGRAPHY_FILE = "calibration/Homography.xml";
    const std::string IMAGE_DIR = "calibration/images";

    bool loadCalibration(cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
        cv::FileStorage fs(INTRINSICS_FILE, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            syslog(LOG_ERR, "Error: Unable to load intrinsic parameters.");
            return false;
        }
        fs["camera_matrix"] >> cameraMatrix;
        fs.release();

        cv::FileStorage fs2(DISTORTION_FILE, cv::FileStorage::READ);
        if (!fs2.isOpened()) {
            syslog(LOG_ERR, "Error: Unable to load distortion coefficients.");
            return false;
        }
        fs2["dist_coeffs"] >> distCoeffs;
        fs2.release();

        return true;
    }


//@brief: Calibrate the camera and compute the homography matrix
//@return: 1 if successful, 0 if chessboard not found,
int calibrateInversePerspectiveMap(const cv::Mat& undistorted, cv::Mat& homography_matrix, std::array<cv::Point2f, 4>& objPts, float horizon) {
    // Chessboard properties
    const cv::Size board_sz = cv::Size(5, 7);  // Internal corners (columns x rows)
    const int board_w = board_sz.width;
    const int board_h = board_sz.height;
    const int square_size = 40; // Chessboard square size (mm or pixels)
    const int board_width = board_w * square_size;
    const int board_height = board_h * square_size;

    // Known points of board. 
    const std::array<cv::Point2f, 4> boardPts = {
        cv::Point2f(320+board_width/2, undistorted.rows -board_height), // Top RIght
        cv::Point2f(320 +board_width/2, undistorted.rows), // bottom right
        cv::Point2f(320-board_width/2, undistorted.rows -board_height), // Top Left
        cv::Point2f(320 - board_width/2, undistorted.rows) // bottom left
    };
    // Detect chessboard corners
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(undistorted, board_sz, corners, 
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (!found) {
        syslog(LOG_ERR, "Chessboard detection failed");
        return 0;
    }

    // Refine detected corners
    cv::cornerSubPix(undistorted, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

    // board points found in the image
    objPts[0] = corners[0];
    objPts[1] = corners[board_w - 1];
    objPts[2] = corners[(board_h - 1) * board_w]; 
    objPts[3] = corners[(board_h * board_w) - 1];

    // Compute homography matrix
    homography_matrix = cv::getPerspectiveTransform(boardPts, objPts);

    // Save homography matrix to file
    syslog(LOG_INFO, "Saving homography matrix to file");
    cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::WRITE);
    fs << "homography_matrix" << homography_matrix;
    fs.release();

    return 1; // Success
}

}


#endif // CALIBRATION_HPP