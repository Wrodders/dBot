/***********************************************************
 *  @file: visionServer.cpp
 *  @brief: Remote Vision Nav processing server
 *  @details: Offload of the vision processing pipeline on remote server
 */

 #include "../inc/vision.hpp"

#define WEBCAM "0"

void print_help(){
    std::cout << "Usage: visionServer <ip> <port>" << std::endl;
}

int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;
    std::cout << "[VISION] Starting Vision Pipeline" << std::endl;
    std::string src(WEBCAM); // Default to webcam
    try{
        src = "udp://"+ std::string(argv[1]) + std::string(":") + std::string(argv[2]);
    }catch(const std::exception& e){
        std::cerr << "Error: " << e.what() << std::endl;
        print_help();
        return -1;
    }
    
    cv::VideoCapture cap(src);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open video source" << std::endl;
        return -1;
    }
    if(src == WEBCAM){
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }


    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Unable to capture frame" << std::endl;
            break;
        }

        // convert theframeto yuv420 
        cv::cvtColor(frame, frame, cv::COLOR_BGR2YUV_I420);
        // Process the frame
        cv::Mat y_plane(frame, cv::Rect(0, 0, frame.cols, frame.rows / 2)); // Y plane
        viz::pipeline(y_plane, homography_matrix, 








        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) break;
    }
    
    return 0;
}