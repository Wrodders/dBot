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




// Define necessary variables
std::vector<std::vector<cv::Point2f>> validChessboards;
const int requiredImages = 1;
const cv::Size board_sz = cv::Size(5, 7); 
const int board_n = board_sz.width * board_sz.height;
const int board_w = board_sz.width;
const int board_h = board_sz.height;
const int square_size = 40;

//@brief: Calibrate the camera and compute the homography matrix
//@return: 1 if successful, 0 if chessboard not found,
int calibrateBirdseye(const cv::Mat& undistorted,cv::Mat& homography_matrix, std::vector<cv::Point2f>& srcPts) {
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(undistorted, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (!found) {return 0;}
    // Refine corner locations
    cv::cornerSubPix(undistorted, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    
    // Get the four outer corners (order matters!)
    // Assuming standard chessboard detection order (left-right, top-bottom)
    srcPts = corners;

    // Define real-world coordinates (in mm or any unit matching square_size)
    float board_width = (board_w - 1) * square_size;  // Number of internal edges
    float board_height = (board_h - 1) * square_size;
    std::vector<cv::Point2f> dstPts(4);
    // Conpue perspecive presinvivig the with of the top of the frame and saling the bottom 
    dstPts[0] = cv::Point2f(320 - board_width/2, undistorted.rows); // Bottom Left
    dstPts[1] = cv::Point2f(0,undistorted.rows-board_height);                // Top Left
    dstPts[2] = cv::Point2f(320+board_width/2, undistorted.rows); // Bottom Right
    dstPts[3] = cv::Point2f(undistorted.cols,  undistorted.rows-board_height); // Top Right  

    // Compute homography matrix
    homography_matrix = cv::getPerspectiveTransform( srcPts, dstPts );
    // Save the homography matrix to a file
    cv::FileStorage fs("calibration/homography_matrix.yml", cv::FileStorage::WRITE);
    fs << "homography_matrix" << homography_matrix;
    fs.release();
    return 1; // Success
}

}


#endif // CALIBRATION_HPP