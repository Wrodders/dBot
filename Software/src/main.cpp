#include "../inc/vision.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>
#include <chrono>

std::mutex velocityMutex;
TargetVelocities latestVel;  // Shared variable to store the latest velocity data

struct state state;

void zmqPublishWorker(zmq::socket_t& publisher) {
    while (!stopFlag) {
        // Sleep to maintain 30Hz sending rate
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Lock mutex to access the latest velocity data safely
        std::lock_guard<std::mutex> lock(velocityMutex);
        
        // Send the latest angular velocity (multiplied by kernel size)
        sendZMQMessage(publisher, 'm',  -0.01f*velScale);
        sendZMQMessage(publisher, 'r', fabs(latestVel.angularVelocity) < 0.8 ? latestVel.angularVelocity : 0.0);

        sendZMQMessage(publisher, 's', (float)AP / 5.0  );
        sendZMQMessage(publisher, 't', (float)KI / 5.0 );
        sendZMQMessage(publisher, 'u', (float)kd / 5.0 );
    }
}
void zmqSubscriberWorker(zmq::socket_t& subscriber) {
    // receive multipart message and update the state with new valeu and timestamp
    // block thread until message is received

    while (!stopFlag) {
        zmq::message_t topic_msg;
        zmq::message_t data_msg;
        int rc = *subscriber.recv(topic_msg, zmq::recv_flags::none);
        if (rc) {
            std::string topic = std::string(static_cast<char*>(topic_msg.data()), topic_msg.size());
            rc = *subscriber.recv(data_msg, zmq::recv_flags::none);
            if (rc) {
                std::string data = std::string(static_cast<char*>(data_msg.data()), data_msg.size());
                state.pitch = std::stof(data);
                state.timestamp = std::chrono::steady_clock::now();
            }
        }
    }
}



int main(int argc, char* argv[]) {
    // IO setup // 
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);

    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://dbot.local:5555");  // Connect to the publisher
    std::string topic = "IMU/PITCH";
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());
    std::thread zmqSubThread(zmqSubscriberWorker, std::ref(subscriber));

    cv::namedWindow("Video Stream", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Video Stream", selectROI, &roiData);

    cv::createTrackbar("Threshold", "Video Stream", &threshold_value, 255);
    cv::createTrackbar("KP", "Video Stream", &AP, 100);
    cv::createTrackbar("KI", "Video Stream", &KI, 100);
    cv::createTrackbar("KD", "Video Stream", &kd, 100);
    cv::createTrackbar("LSCALE", "Video Stream", &velScale, 20);

    // VIDEO CAPTURE SETUP
    cv::VideoCapture cap("udp://192.168.0.32:5000");
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1); //now the opencv buffer just one frame.
    cv::Mat frame; // input frame
    cv::Mat result; // Processed frame
    

    std::cout << "Starting Vision Loop\n";
    while (!stopFlag) {
        if (!cap.read(frame)) {
            continue;
        }
        TargetVelocities trgtVel = method2(frame, result, roiData, state);
        updateROI(frame, roiData);

        if (!result.empty()) {cv::imshow("Processed Video", result);}
        cv::imshow("Video Stream", frame);
        // Handle key events, exit if 'q' or 'ESC' is pressed
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            stopFlag = true;
        }
    }
    zmqSubThread.join();
    cv::destroyAllWindows();
    return 0;
}