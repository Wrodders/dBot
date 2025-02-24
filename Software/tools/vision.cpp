/************************************************************************************
 *  @file    vision.cpp
 *  @brief   Monocular Vision Trajectory Generations
 *  @date    2025-01-05
 *  @version 0.01
 * 
 * Gstreamer Pipeline:
 * Reads frames in YUV from a pipe.
 * For each captured frame generates a continuous trajectory as a set of linear and angular velocity commands
 * Performs interactive calibration of the camera using OpenCV and saves the calibration parameters to a file
 * Publishes the processed frames to the output pipe for visualization
 * 
 * 
 * Command Server 
 * Implements a pub-rpc client over ZMQ, Listens for commands on VISION topic over IPC; 
 * these modify runtime parameters through config managers
 * Outputs CMD_RET/VISION messages to the message socket
 * 
 * Target Frame Rate: 25 Hz
 * Output frame for visualization
 * 
 *         640px           
 * |---------------------|
 * | color| Birds| Hist  | 
 * |------| -----|-------| 480px
 * | ROI  | Edge | window|
 * ----------------------|
 ************************************************************************************/


#include "../inc/vision.hpp"
#include <deque>

#define NODE_NAME "VISION"





int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;
    std::cout << "[VISION] Starting Vision Pipeline" << std::endl;
    //  -------------- Video IO Setup  ----------------- //
    mkfifo(viz::INPUT_PIPE, 0666); 
    mkfifo(viz::OUTPUT_PIPE, 0666); 
    // Command to capture video using rpicam-vid with YUV420p output
    FILE* in_pipe = fopen(viz::INPUT_PIPE, "r");
    if (!in_pipe) {
        std::cerr << "Failed to open pipe to video input!" << std::endl;
        return -1;
    }
   
    FILE* out_pipe = fopen(viz::OUTPUT_PIPE, "w");
    if (!out_pipe) {
        std::cerr << "Failed to open pipe to video output!" << std::endl;
        return -1;
    }
    // -------------- Runtime Parameters ----------------- //
    std::array<std::atomic<float>, viz::NUM_PARAMS> params;
    ParameterMap param_map("configs/robotConfig.json", NODE_NAME);
    param_map.register_parameter(viz::P_HRZ_HEIGHT, params[viz::P_HRZ_HEIGHT], viz::val_hrz_height);
    param_map.register_parameter(viz::P_TRFM_PAD, params[viz::P_TRFM_PAD], viz::val_trfm_pad);
    param_map.register_parameter(viz::P_EDGE, params[viz::P_EDGE], viz::val_edge);
    param_map.register_parameter(viz::P_PROG_MODE, params[viz::P_PROG_MODE], viz::val_prog_mode);

    // initialize the parameters
    params[viz::P_HRZ_HEIGHT].store(viz::HEIGHT / 2);
    params[viz::P_TRFM_PAD].store(viz::WIDTH / 2);
    params[viz::P_EDGE].store(viz::P_CANNY);
    params[viz::P_PROG_MODE].store(viz::M_RUN);
    

    


    // ------------- Command Server Coms Thread ----------------- //
    std::thread cmd_thread(viz::command_server, std::ref(param_map));
    // -------------- Vision Pipeline Allocations ----------------- //
    std::array<uint8_t, viz::FRAME_SIZE> yuv_buffer; // serialized frame buffer
    cv::Mat homography_matrix = cv::Mat::eye(3, 3, CV_64F); // identity matrix

    while(viz::_exit_trig == false) {
        size_t bytes_read = fread(yuv_buffer.data(), 1, yuv_buffer.size(), in_pipe);
        if (bytes_read != viz::FRAME_SIZE) {
            std::cerr << "[VISION] Error: Unable to read a complete frame." << std::endl;
            continue; // skip frame if not complete
        }
        cv::Mat y_plane(viz::HEIGHT, viz::WIDTH, CV_8UC1, yuv_buffer.data()); // zero-copy
        float progmode;
        (void) param_map.get_value(viz::P_PROG_MODE, progmode);
        switch((int)progmode){
            case viz::M_CALIBRATE:
                // Run Camera Intrinsic Calibration
                break;
            case viz::M_INIT:
                // Initialize the pipeline
                break;
            case viz::M_PRE:
                // Run Preprocessing Calibration
                break;
            case viz::M_POST:
                // Checks for calibration parameters
                // Checks 
                break;
            case viz::M_RUN:
                if(viz::pipeline(y_plane, homography_matrix, param_map) == -1){
                    viz::_exit_trig.store(true); // force exit
                    break;}
                break;
            default:
                break;
        }
        // ------------ Serialize Output Frame ------------ //
        std::memcpy(yuv_buffer.data(), y_plane.data, y_plane.total()); // overwrite the y_plane
        uint8_t* u_plane = yuv_buffer.data() + (viz::WIDTH * viz::HEIGHT);
        uint8_t* v_plane = u_plane + (viz::WIDTH * viz::HEIGHT / 4); // 
        std::fill(u_plane, u_plane + (viz::WIDTH * viz::HEIGHT / 4), 128); // grayout the U and V planes
        std::fill(v_plane, v_plane + (viz::WIDTH * viz::HEIGHT / 4), 128);
        try {
            viz::write_frame(out_pipe, yuv_buffer);
        } catch (const std::exception& e) {
            fmt::print("[{}] Error: {}\n", "VISION", e.what());
        }
    }
    viz::_exit_trig.store(true); // force exit
    cmd_thread.join(); 
    fclose(in_pipe);
    fclose(out_pipe);
    fmt::print("[{}] Exiting\n", NODE_NAME);
    return 0;
}
