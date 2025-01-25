#include "../common/common.h"
#include <iostream>
#include <sys/time.h>


#include <queue>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <unistd.h>
#include <string.h>
#include <sys/fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/select.h>
#include <thread>

#include <csignal>


#include "../../Firmware/inc/mcuComs.h"

#define MAX_STRING_LENGTH 30  // Maximum string length for INFO, DEBUG, ERROR

struct MotorState {
    struct IMU {
        float pitch;
        float roll;
    } imu;

    struct Odometry {
        float lw;
        float rw;
        float linVel;
        float angVel;
        float xPos;
        float yPos;
        float theta;
    } odom;



    time_t timestamp;
};




void printMotorStateTable(struct MotorState &motorState) {
    // ANSI escape code to clear the screen
    printf("\033[H\033[J");
    // Table Header
    printf("Motor State Table:\n");
    printf("+-----------------------+------------------------+\n");
    printf("| %-21s | %-22s |\n", "Variable", "Value");
    printf("+-----------------------+------------------------+\n");
    // Table Rows (Formatted for fixed column width)
    printf("| %-21s | %-22.2f |\n", "IMU Pitch",            motorState.imu.pitch);
    printf("| %-21s | %-22.2f |\n", "IMU Roll",             motorState.imu.roll);
    printf("| %-21s | %-22.2f |\n", "Odometry Left Wheel",  motorState.odom.lw);
    printf("| %-21s | %-22.2f |\n", "Odometry Right Wheel", motorState.odom.rw);
    printf("| %-21s | %-22.2f |\n", "Odometry Linear Vel",  motorState.odom.linVel);
    printf("| %-21s | %-22.2f |\n", "Odometry Angular Vel", motorState.odom.angVel);
    printf("| %-21s | %-22.2f |\n", "Odometry X Position",  motorState.odom.xPos);
    printf("| %-21s | %-22.2f |\n", "Odometry Y Position",  motorState.odom.yPos);
    printf("| %-21s | %-22.2f |\n", "Odometry Theta",       motorState.odom.theta);
    printf("| %-21s | %-22.2f |\n", "Control TL",           motorState.cntrl.targetLeftWheelRPM);
    printf("| %-21s | %-22.2f |\n", "Control TR",           motorState.cntrl.targetRightWheelRPM);
    printf("| %-21s | %-22.2f |\n", "Control UL",           motorState.cntrl.ul);
    printf("| %-21s | %-22.2f |\n", "Control UR",           motorState.cntrl.ur);
    printf("| %-21s | %-22.2f |\n", "Control UB",           motorState.cntrl.ub);
    printf("| %-21s | %-22.2f |\n", "Control UA",           motorState.cntrl.ua);
    printf("| %-21s | %-22.2f |\n", "Control UV",           motorState.cntrl.uv);

    // Handle timestamp formatting
    char timeBuffer[26];
    struct tm* tm_info = localtime(&motorState.timestamp);
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", tm_info);

    printf("| %-21s | %-22s |\n", "Timestamp", timeBuffer);
    // Table Footer
    printf("+-----------------------+------------------------+\n");
}


// Global flag to handle graceful exit
volatile bool keepRunning = true;

void signalHandler(int signum) {
    keepRunning = false;
    std::cout << "\nExiting program...\n";
}

int main(int argc, char* argv[]) {
    // Argument parsing
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <param_file.csv> <serial_port>\n";
        return EXIT_FAILURE;
    }

    const char* paramFile = argv[1];
    const char* serialPort = argv[2];

    try {
        // Register signal handler for graceful termination
        std::signal(SIGINT, signalHandler);

        // Main loop
        while (keepRunning) {
            // Fetch the current motor state
            

            // Print the motor state table
            printMotorStateTable(*motorState);

            // Delay to avoid excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
