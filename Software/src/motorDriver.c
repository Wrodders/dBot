#include "../common/common.h"
#include "../inc/motorDriver.h"

struct MotorState motorState;

// Read thread function (reads line-by-line and packs data into MotorState)
void* read_serial_worker(void *arg) {
    int fd = *((int *)arg);
    char buffer[256];
    FILE *fp = fdopen(fd, "r");  // Open file stream for reading
    if (fp == NULL) {
        perror("Failed to associate file stream");
        return NULL;
    }
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  // Asynchronous cancellation
    while (!stop) {
        if (fgets(buffer, sizeof(buffer), fp) != NULL) {
            if (buffer[0] == '<') {  // Check for start of message
                switch(buffer[1]-'a'){
                    case PUB_STATE:
                        sscanf(&buffer[2], 
                        "%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f",
                        &motorState.imu.pitch, &motorState.imu.roll,
                        &motorState.odom.lw, &motorState.odom.rw,
                        &motorState.odom.linVel, &motorState.odom.angVel,
                        &motorState.odom.xPos, &motorState.odom.yPos, &motorState.odom.theta,
                        &motorState.cntrl.tl, &motorState.cntrl.tr,
                        &motorState.cntrl.ul, &motorState.cntrl.ur, 
                        &motorState.cntrl.ub, &motorState.cntrl.ua, &motorState.cntrl.uv);
                        motorState.timestamp = time(NULL);
                        break;   
                    case PUB_INFO:
                        sscanf(&buffer[2], "%s", motorState.info);
                        break;
                    case PUB_DEBUG:
                        sscanf(&buffer[2], "%s", motorState.debug);
                        break;
                    case PUB_ERROR:
                        sscanf(&buffer[2], "%s", motorState.error);
                        break;
                    case PUB_CMD_RET:
                        sscanf(&buffer[2], "%f", &motorState.cmdRet);
                        break;                        
                    default:
                        printf("Received message: %s", buffer);
                        break;
                }
            }
        }
    }
    fclose(fp);
    return NULL;
}

#define MAX_STRING_LENGTH 20  // Maximum string length for INFO, DEBUG, ERROR

void printMotorStateTable() {
    // ANSI escape code to clear the screen
    printf("\033[H\033[J");

    // Table Header
    printf("Motor State Table:\n");
    printf("+-----------------------+------------------------+\n");
    printf("| %-21s | %-22s |\n", "Variable", "Value");
    printf("+-----------------------+------------------------+\n");

    // Table Rows (Formatted for fixed column width)
    printf("| %-21s | %-22.2f |\n", "IMU Pitch", motorState.imu.pitch);
    printf("| %-21s | %-22.2f |\n", "IMU Roll", motorState.imu.roll);
    printf("| %-21s | %-22.2f |\n", "Odometry Left Wheel", motorState.odom.lw);
    printf("| %-21s | %-22.2f |\n", "Odometry Right Wheel", motorState.odom.rw);
    printf("| %-21s | %-22.2f |\n", "Odometry Linear Vel", motorState.odom.linVel);
    printf("| %-21s | %-22.2f |\n", "Odometry Angular Vel", motorState.odom.angVel);
    printf("| %-21s | %-22.2f |\n", "Odometry X Position", motorState.odom.xPos);
    printf("| %-21s | %-22.2f |\n", "Odometry Y Position", motorState.odom.yPos);
    printf("| %-21s | %-22.2f |\n", "Odometry Theta", motorState.odom.theta);
    printf("| %-21s | %-22.2f |\n", "Control TL", motorState.cntrl.tl);
    printf("| %-21s | %-22.2f |\n", "Control TR", motorState.cntrl.tr);
    printf("| %-21s | %-22.2f |\n", "Control UL", motorState.cntrl.ul);
    printf("| %-21s | %-22.2f |\n", "Control UR", motorState.cntrl.ur);
    printf("| %-21s | %-22.2f |\n", "Control UB", motorState.cntrl.ub);
    printf("| %-21s | %-22.2f |\n", "Control UA", motorState.cntrl.ua);
    printf("| %-21s | %-22.2f |\n", "Control UV", motorState.cntrl.uv);

    // Handle timestamp formatting
    char timeBuffer[26];
    struct tm* tm_info = localtime(&motorState.timestamp);
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", tm_info);

    printf("| %-21s | %-22s |\n", "Timestamp", timeBuffer);

    // Ensure INFO, DEBUG, ERROR fields are within limit and aligned
    printf("| %-21s | %-22.20s |\n", "INFO", motorState.info);  // Truncated to 20 chars
    printf("| %-21s | %-22.20s |\n", "DEBUG", motorState.debug);  // Truncated to 20 chars
    printf("| %-21s | %-22.20s |\n", "ERROR", motorState.error);  // Truncated to 20 chars

    // CMD_RET formatted with two decimal places
    printf("| %-21s | %-22.2f |\n", "CMD_RET", motorState.cmdRet);

    // Table Footer
    printf("+-----------------------+------------------------+\n");
}


// Signal handler for SIGINT (Ctrl+C)
void handle_sigint(int sig) {
    stop = 1;  // Set the flag to stop threads
    printf("\nCaught signal %d, exiting gracefully...\n", sig);
}

int main() {
    signal(SIGINT, handle_sigint);

    int fd = configure_serial_port(SERIAL_PORT);
    pthread_t read_threadID;
    if (pthread_create(&read_threadID, NULL, read_serial_worker, &fd) != 0) {
        perror("Failed to create read thread");
        close(fd);
        exit(1);
    }

    // Main loop to keep the program running
    while (!stop) {
        printf("IMU: %.2f, %.2f\n", motorState.imu.pitch, motorState.imu.roll);
        printMotorStateTable();
        usleep(0.1e6);
    }
    // Wait for threads to finish (join them) before exiting
    pthread_cancel(read_threadID);
    //pthread_join(read_threadID, NULL);
    close(fd);  //
}