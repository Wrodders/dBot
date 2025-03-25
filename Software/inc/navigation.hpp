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

//@brief: Computes velocity references from prominence, confidence and drivability
void pathFollower(const int peakIdx, const int curveWidth, const int curveConfidance, ParameterMap& param_map) {
    // Normalize the cross-track error to -1 to 1
    float crossTrackError = (peakIdx - 320) / 320.0f;
    bool trackLost = (curveConfidance < 50 || curveWidth > 600);
    float curveFactor =  curveWidth / 600.0f; 
    float confidenceFactor = curveConfidance / 255.0f;

    // Trajectory object to hold speed and w_rate
    Trajectory twist;

    if (trackLost) {
        // If track is lost, stop movement and set no rotation
        twist.w_rate = 0.0f;
        twist.speed = 0.0f;
    } else {    
        float maxSpeed;
        (void) param_map.get_value(viz::P_MAX_VEL, maxSpeed);

        twist.speed = maxSpeed * (1 - curveFactor) * confidenceFactor;
        twist.speed = std::clamp(twist.speed, 0.0f, maxSpeed);

        float widthScaling = 1.0f + curveFactor;

        float errsqr = crossTrackError * crossTrackError;
        
        twist.w_rate = std::clamp(errsqr * 0.6f * widthScaling , -0.8f, 0.8f);
    }
    // Push the trajectory twist to the queue for the robot control
    {
        std::unique_lock<std::mutex> lock(_twist_mutex);
        _twist_queue.push(twist);
        lock.unlock();
        _twist_cv.notify_one();
    }
}
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