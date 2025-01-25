#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <zmq.hpp>

double prevError = 0.0;
double integral = 0.0;
// Maximum and minimum values for the sliders to ensure the normalization between -1 and 1
const int maxSliderValue = 100;
const int minSliderValue = -100;

// Target angular velocity set by the command
double targetAngularVelocity = 0.0;

// PID controller function to compute control output


struct ROIData {
    std::vector<cv::Point> points; // Store selected points for the trapezium
    bool drawing = false;
    bool roiSelected = false;
    int pointCount = 0; // Counter to track the number of points selected
};

class Vision {
public:
    Vision() 
        : thresholdValue(63), intensityThreshold(50), numSamples(3), kernelSize(5), stopFlag(false), 
          context(1), publisher(context, ZMQ_PUB), Kp(0), Ki(0), Kd(0) {
        // Bind the publisher to TCP port 5556
        publisher.bind("ipc:///tmp/vision_cmd");

        cv::namedWindow("Original Frame");
        cv::namedWindow("Processed ROI");

        // Create trackbars for PID control parameters
        cv::createTrackbar("Kp", "Processed ROI", &KpInt, 100, onKpChange, this);
        cv::createTrackbar("Ki", "Processed ROI", &KiInt, 100, onKiChange, this);
        cv::createTrackbar("Kd", "Processed ROI", &KdInt, 100, onKdChange, this);
        cv::createTrackbar("Threshold", "Processed ROI", &thresholdValue, 255, onThresholdChange, this);
        cv::createTrackbar("Intensity Threshold", "Processed ROI", &intensityThreshold, 255, onIntensityThresholdChange, this);
        cv::createTrackbar("Num Samples", "Processed ROI", &numSamples, 25, onNumSamplesChange, this);
        cv::createTrackbar("Kernel Size", "Processed ROI", &kernelSize, 25, onKernelSizeChange, this);
    }

    void run() {
        // VIDEO CAPTURE SETUP
        cv::VideoCapture cap("udp://192.168.0.32:5000");
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1); //now the opencv buffer just one frame.
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video stream." << std::endl;
            return;
        }

        cv::setMouseCallback("Original Frame", onMouse, &roiData);

        while (!stopFlag) {
            cap.read(frame);
            if (frame.empty()) continue;

            overlayFrame = frame.clone(); // Create a separate overlay frame
            updateROI();
            processFrame();
            cv::imshow("Original Frame", overlayFrame); // Display overlayFrame with ROI points
            if (!processedROI.empty()) cv::imshow("Processed ROI", processedROI);

            if (cv::waitKey(1) == 'q') stopFlag = true;
        }
    }

private:
    cv::Mat frame, processedROI, overlayFrame;
    ROIData roiData;
    int thresholdValue, intensityThreshold, numSamples, kernelSize;
    bool stopFlag;

    int KpInt, KiInt, KdInt;  // For the trackbars
    double Kp, Ki, Kd;        // Actual PID variables used in computation
    double controlOutput;

    zmq::context_t context; // ZMQ context
    zmq::socket_t publisher; // ZMQ publisher socket

    static void onMouse(int event, int x, int y, int flags, void* userdata) {
        auto* data = static_cast<ROIData*>(userdata);
        if (event == cv::EVENT_LBUTTONDOWN) {
            if (data->pointCount == 4) {
                data->points.clear();
                data->pointCount = 0;
                data->roiSelected = false;
            }
            data->points.push_back(cv::Point(x, y));
            data->pointCount++;
            if (data->pointCount == 4) data->roiSelected = true;
        }
    }

    static void onThresholdChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->thresholdValue = value;
    }

    static void onIntensityThresholdChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->intensityThreshold = value;
    }

    static void onNumSamplesChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->numSamples = value > 0 ? value : 1;
    }

    static void onKernelSizeChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->kernelSize = (value % 2 == 1 && value > 0) ? value : 5;
    }

    // Callbacks to handle slider changes and normalize values between -1 and 1
    static void onKpChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->Kp = (value / 25.0);  // Normalize between -1 and 1
    }

    static void onKiChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->Ki = (value / 1000.0);  // Normalize between -1 and 1
    }

    static void onKdChange(int value, void* userdata) {
        auto* vision = static_cast<Vision*>(userdata);
        vision->Kd = (value / 25.0);  // Normalize between 0 and 4
    }

    void updateROI() {
        if (roiData.pointCount > 0) {
            for (const auto& point : roiData.points) {
                cv::circle(overlayFrame, point, 5, cv::Scalar(0, 0, 255), -1);
            }
            if (roiData.pointCount == 4) {
                cv::polylines(overlayFrame, roiData.points, true, cv::Scalar(0, 255, 0), 2);
            }
        }
    }


    double computePID(double error) {
        integral += error;  // Accumulate the error for the integral term
        // clamp the integral term to prevent windup
        if (integral > 1) integral = 1;
        else if (integral < -1) integral = -1;
        double derivative = error - prevError;  // Calculate the derivative term
        double output = Kp * error + Ki * integral + Kd * derivative;  // PID output
        prevError = error;  // Save the error for the next iteration
        // Clamp the output to the range [-1, 1]
        if (output > 3) output = 3;
        else if (output < -3) output = -3;
        return output;
    }
    
   void processFrame() {
        if (!roiData.roiSelected || roiData.pointCount != 4) return;

        cv::Mat roi;
        applyROIWithPadding(10, 10, roi);

        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        cv::threshold(roi, roi, thresholdValue, 255, cv::THRESH_BINARY);
        cv::GaussianBlur(roi, roi, cv::Size(kernelSize, kernelSize), 0);

        processedROI = roi.clone();

        std::vector<cv::Point> rowMeans = computeRowMeans(roi);
        for (auto& point : rowMeans) {
            point.y = frame.rows - point.y; // Invert the y-coordinate
            point.x -= frame.cols / 2;
        }

        // Ensure there are enough points to compute the average of the last 4 points
        if (rowMeans.size() >= 10) {
            // Calculate the average of the first 4 points in the last 4 rows of the grid
            cv::Point avgPoint(0, 0);
            for (int i = 0; i < 10; ++i) {
                avgPoint += rowMeans[rowMeans.size() - 1 - i];
            }
            avgPoint.x /= 10;
            avgPoint.y /= 10;

            // draw average point in green
            cv::circle(overlayFrame, cv::Point(avgPoint.x + frame.cols / 2, frame.rows - avgPoint.y), 5, cv::Scalar(0, 255, 0), -1);

            // transfom the average points cooriantes to the center bottom of the frame not the top left

            for (auto& point : rowMeans) {
                point.y = frame.rows - point.y; // Invert the y-coordinate
                point.x -= frame.cols / 2; // Center the x-coordinate
            }

            // Compute the error as the horizontal distance from the center (x = 0)
            double error = (0.0 - avgPoint.x) / frame.cols;
            // scale the y value such that the robot moves faster when the point is closer to the top of the frame
            double speed = -0.2 * avgPoint.y / frame.rows;
            // Apply the PID controller to compute the angular velocity
            
            // apply a low pass filer to smooth the output and delay it to account for the delay in the robot
            controlOutput= computePID(error) ;

            // Send the calculated angular velocity over ZMQ
            sendZMQMessage('r', controlOutput);
        }
    }

    void sendZMQMessage(char id, double val) {
        // Define the topic with 'SERIAL/' prefix
        std::string topic = "SERIAL/";

        // Format the message with '<' at the beginning and '>' at the end, including the id and value
        std::ostringstream oss;
        oss << "<b" << id << val << "\n";
        std::string message = oss.str();

        // Create ZeroMQ message objects for the topic and actual message
        zmq::message_t topicMessage(topic.c_str(), topic.size());
        zmq::message_t zmqMessage(message.size());


        std::cout << "Sending: " << topic << message << std::endl;

        // Copy the formatted message into the ZeroMQ message
        memcpy(zmqMessage.data(), message.c_str(), message.size());

        // Send the topic and message separately to publish them
        publisher.send(topicMessage, zmq::send_flags::sndmore);
        publisher.send(zmqMessage, zmq::send_flags::none);
    }

    void applyROIWithPadding(int topPadding, int bottomPadding, cv::Mat& result) {
        cv::Rect boundingRect = cv::boundingRect(roiData.points);
        int top = std::max(boundingRect.y - topPadding, 0);
        int bottom = std::min(boundingRect.y + boundingRect.height + bottomPadding, frame.rows);
        int left = std::max(boundingRect.x, 0);
        int right = std::min(boundingRect.x + boundingRect.width, frame.cols);

        cv::Rect adjustedROI(left, top, right - left, bottom - top);

        cv::Mat mask = cv::Mat::zeros(adjustedROI.size(), CV_8UC1);
        std::vector<cv::Point> shiftedPoints;
        for (const auto& point : roiData.points) {
            shiftedPoints.emplace_back(point.x - adjustedROI.x, point.y - adjustedROI.y);
        }
        cv::fillPoly(mask, {shiftedPoints}, cv::Scalar(255));

        frame(adjustedROI).copyTo(result, mask);
    }
    // return vecotr of row means

    std::vector<cv::Point> computeRowMeans(cv::Mat& roi) {
        int divisionHeight = roi.rows / numSamples;
        int divisionWidth = roi.cols / numSamples;
        std::vector<cv::Point> rowMeans;
        for (int i = 0; i < numSamples; ++i) {
            std::vector<cv::Point> rowCentroids;
            for (int j = 0; j < numSamples; ++j) {
                cv::Rect gridRect(j * divisionWidth, i * divisionHeight, divisionWidth, divisionHeight);
                cv::Mat grid = roi(gridRect);

                if (cv::mean(grid)[0] > intensityThreshold) {
                    cv::Moments moments = cv::moments(grid, true);
                    if (moments.m00 > 0) {
                        double cx = j * divisionWidth + moments.m10 / moments.m00;
                        double cy = i * divisionHeight + moments.m01 / moments.m00;
                        rowCentroids.emplace_back(cx, cy);
                        cv::circle(overlayFrame, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
                    }
                }
            }

            if (!rowCentroids.empty()) {
                cv::Point meanPoint = computeMean(rowCentroids);
                rowMeans.push_back(meanPoint);
                cv::circle(overlayFrame, meanPoint, 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        drawGridLines(overlayFrame, numSamples, divisionHeight, divisionWidth);
        return rowMeans;
    }

    cv::Point computeMean(const std::vector<cv::Point>& points) {
        cv::Point sum(0, 0);
        for (const auto& point : points) {
            sum += point;
        }
        return cv::Point(sum.x / points.size(), sum.y / points.size());
    }

    void drawGridLines(cv::Mat& image, int numDivisions, int divisionHeight, int divisionWidth) {
        for (int i = 1; i < numDivisions; ++i) {
            cv::line(image, cv::Point(0, i * divisionHeight), cv::Point(image.cols, i * divisionHeight), cv::Scalar(255, 255, 255), 1);
            cv::line(image, cv::Point(i * divisionWidth, 0), cv::Point(i * divisionWidth, image.rows), cv::Scalar(255, 255, 255), 1);
        }
    }

    

};

int main() {
    Vision vision;
    vision.run();
    return 0;
}
