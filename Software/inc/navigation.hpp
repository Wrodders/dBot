#ifndef NAV_HPP
#define NAV_HPP
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
#include "../inc/vision.hpp"



namespace nav {

struct Trajectory {
    float w_rate;
    float speed;
};

std::queue<Trajectory> _twist_queue;
std::mutex _twist_mutex;
std::condition_variable _twist_cv;

//@Brief: Simple Point Tracking Algorithm - Bang Bang
//@Description: Computes the cross-track error based on the histogram of the image
//              Sets angular velocity opposing the error. 
//@Note        Sets linear velocity to max speed parameter
void bangbang(const int peakIdx, ParameterMap& param_map) {
    float crossTrackError = 320 - peakIdx;
    crossTrackError /= 320 / 2;
    crossTrackError = std::clamp(crossTrackError, -0.6f, 0.6f);
    float maxSpeed;
    (void) param_map.get_value(viz::P_MAX_VEL, maxSpeed);
    struct Trajectory twist = {.w_rate = -crossTrackError, .speed = maxSpeed};
    // Push the trajectory twist to the queue for the robot control
    {
        std::unique_lock<std::mutex> lock(_twist_mutex);
        _twist_queue.push(twist);
        lock.unlock();
        _twist_cv.notify_one();
    }
}

//@Brief: Slows down speed based on curve width and confidance, 
void slowOnCurves(const int peakIdx, const int curveWidth, const int curveConfidance, ParameterMap& param_map) {
    // Normalize the cross-track error to -1 to 1
    float crossTrackError = viz::WIDTH / 2 - peakIdx;
    crossTrackError /= viz::WIDTH / 2;
    crossTrackError = std::clamp(crossTrackError, -0.2f, 0.2f);
    bool trackLost = (curveConfidance < 55 || curveWidth > 600);
    // Trajectory object to hold speed and w_rate
    Trajectory twist;
    if (trackLost) {
        // If track is lost, stop movement and set no rotation
        twist.w_rate = 0.0f;
        twist.speed = 0.0f;
    } else {    
        float curveFactor =  curveWidth / 600.0f;  // normalize to 0-1
        float confidenceFactor = curveConfidance / 200.0f;
        float maxSpeed;
        (void) param_map.get_value(viz::P_MAX_VEL, maxSpeed);

        twist.speed = maxSpeed * (1 - curveFactor) * confidenceFactor;
        twist.speed = std::clamp(twist.speed, 0.0f, maxSpeed);
        twist.w_rate = -crossTrackError;      
    }
    // Push the trajectory twist to the queue for the robot control
    {
        std::unique_lock<std::mutex> lock(_twist_mutex);
        _twist_queue.push(twist);
        lock.unlock();
        _twist_cv.notify_one();
    }
}


// @brief: pure pursuit algorithm

//@brief: Trajectory Generation Server
//@description: Publishes twist trajectory commands 
void trajGenServer(){
    zmq::context_t context(1);
    zmq::socket_t traj_pubsock(context, zmq::socket_type::pub);
    traj_pubsock.set(zmq::sockopt::linger, 0);
    traj_pubsock.bind("ipc:///tmp/vizcmds");
    while(true){
        std::unique_lock<std::mutex> lock(_twist_mutex);
        _twist_cv.wait(lock, []{return !_twist_queue.empty();});
        Trajectory twist = _twist_queue.front();
        _twist_queue.pop();
        lock.unlock();
        // Build command messages
        std::string msg = "<BR" + std::to_string(twist.w_rate) + "\n";
        traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
        traj_pubsock.send(zmq::message_t(msg.c_str(), msg.size()), zmq::send_flags::none);
        std::string msg2 = "<BM" + std::to_string(twist.speed) + "\n";
        traj_pubsock.send(zmq::message_t("TWSB", 5), zmq::send_flags::sndmore);
        traj_pubsock.send(zmq::message_t(msg2.c_str(), msg2.size()), zmq::send_flags::none);
    }   
}

}

#endif // NAV_HPP