/***************************************
 * @file    calibration.cpp
 * @brief   Camera calibration tool
 * 
 * 
 * 
 */

 
#include "../inc/calibration.hpp"

int main(int argc, char** argv) {
    // Validate input arguments
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <board_w> <board_h>" << std::endl;
        return -1;
    }

    int board_w = std::atoi(argv[1]);
    int board_h = std::atoi(argv[2]);
    if (board_w <= 0 || board_h <= 0) {
        std::cerr << "Error: Invalid board size values." << std::endl;
        return -1;
    }
    int board_n = board_w * board_h;
    cv::Size board_sz(board_w, board_h);

    // Prepare 3D object points for chessboard corners
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_h; i++) {
        for (int j = 0; j < board_w; j++) {
            objp.emplace_back(static_cast<float>(j), static_cast<float>(i), 0.0f);
        }
    }

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    // Get list of image file names
    std::vector<cv::String> image_files;
    cv::glob(calib::IMAGE_DIR + std::string("/*.jpg"), image_files);  
    if (image_files.empty()) {
        std::cerr << "Error: No images found in '" << calib::IMAGE_DIR << "'." << std::endl;
        return -1;
    }

    cv::namedWindow("Chessboard", cv::WINDOW_AUTOSIZE);
    int successes = 0;
    cv::Mat img, gray;

    for (const auto& file : image_files) {
        img = cv::imread(file);
        if (img.empty()) {
            std::cerr << "Warning: Failed to load image: " << file << std::endl;
            continue;
        }

        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, board_sz, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS
        );
        if (found) {
            cv::cornerSubPix(
                gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1)
            );
            cv::drawChessboardCorners(img, board_sz, corners, found);
            cv::imshow("Chessboard", img);
            cv::waitKey(500);

            if (corners.size() == static_cast<size_t>(board_n)) {
                image_points.push_back(corners);
                object_points.push_back(objp);
                successes++;
                std::cout << "Chessboard found in " << file 
                          << " (" << successes << "/" << image_files.size() << ")" 
                          << std::endl;
            }
        } else {
            std::cerr << "Warning: Chessboard not found in " << file << std::endl;
        }
    }
    cv::destroyWindow("Chessboard");

    if (successes < 5) { // Require at least 5 valid images for calibration
        std::cerr << "Error: Insufficient valid images (" << successes << "). Calibration aborted." << std::endl;
        return -1;
    }

    // Perform camera calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(
        object_points, image_points, gray.size(),
        cameraMatrix, distCoeffs, rvecs, tvecs
    );
    std::cout << "Calibration RMS error: " << rms << std::endl;

    // Save calibration parameters
    {
        cv::FileStorage fs(calib::INTRINSICS_FILE, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cerr << "Error: Unable to save intrinsic parameters." << std::endl;
            return -1;
        }
        fs << "camera_matrix" << cameraMatrix;
    }

    {
        cv::FileStorage fs(calib::DISTORTION_FILE, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cerr << "Error: Unable to save distortion coefficients." << std::endl;
            return -1;
        }
        fs << "dist_coeffs" << distCoeffs;
    }

    std::cout << "Calibration successful!\nCamera Matrix:\n"
              << cameraMatrix << "\nDistortion Coefficients:\n"
              << distCoeffs << std::endl;

    // Display undistorted images
    cv::namedWindow("Undistorted", cv::WINDOW_AUTOSIZE);
    cv::Mat undistorted;
    for (const auto& file : image_files) {
        img = cv::imread(file);
        if (img.empty()) {
            std::cerr << "Warning: Skipping unreadable image " << file << std::endl;
            continue;
        }
        cv::undistort(img, undistorted, cameraMatrix, distCoeffs);
        cv::imshow("Undistorted", undistorted);
        if (cv::waitKey(500) == 27)  // Exit on ESC key
            break;
    }

    cv::destroyAllWindows();
    return 0;
}
