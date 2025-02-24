/***********************************************
* @file calibrate.cpp
* @brief Monocular Vision Calibration Zhang's Method
* @details Saves the calibration parameters to a file
*/

#include "../common/coms.hpp"
#include "../common/common.hpp"
#include "../inc/vision.hpp"


int main(int argc, char *argv){


    std::cout << "[VISION] Starting Vision Pipeline" << std::endl;
    //  -------------- Video IO Setup  ----------------- //
    mkfifo(viz::INPUT_PIPE, 0666); 
    mkfifo(viz::OUTPUT_PIPE, 0666); 
    // Command to capture video using rpicam-vid with YUV420p output
    FILE* in_pipe = fopen(INPUT_PIPE, "r");
    if (!in_pipe) {
        std::cerr << "Failed to open pipe to video input!" << std::endl;
        return -1;
    }
   
    FILE* out_pipe = fopen(OUTPUT_PIPE, "w");
    if (!out_pipe) {
        std::cerr << "Failed to open pipe to video output!" << std::endl;
        return -1;
    }


    return 0;
}