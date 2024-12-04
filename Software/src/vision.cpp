#include <../inc/vision.hpp>

// Function to display help text
void displayHelp() {
    std::cout << "Usage: program_name [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --help           Display this help message\n";
    std::cout << "  --source <path>  Specify the source file (default: videoData.mp4)\n";
}

// Function to parse the arguments
std::string handleCLI(int argc, char* argv[]) {
    std::string source = "videoData.mp4";  // Default value for source file
    
    // Check for --help flag
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            displayHelp();
            exit(0);  // Exit after showing help
        }
    }

    // Parse --source flag and its argument
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--source" && i + 1 < argc) {
            source = argv[i + 1];
            i++;  // Skip next argument, it's the source file path
        }
    }
    
    return source;
}

int main(int argc, char* argv[]) {
    // Parse the command-line arguments
    udpAddress = handleCLI(argc, argv);
    // ZMQ publisher setup
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    //setupZMQPublisher(context, publisher);

    std::thread zmqThread(zmqSender, std::ref(publisher));

    if (!openFFmpegProcess(udpAddress)) {
            std::cerr << "Error: Could not open UDP stream.\n";
            return -1;
        }

    cv::namedWindow("Original Video", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Original Video", selectROI, &roiData);
    setupTrackBars();

    cv::Mat frame, resultFrame;
    std::thread readThread(readFramesAsync, std::ref(frame));

    while (!stopFlag) {
        if (frameReady) {
            method2(frame, resultFrame, roiData);
            updateROI(frame, roiData);
            if (!resultFrame.empty()) {
                cv::imshow("Processed Video", resultFrame);
            }
            cv::imshow("Original Video", frame);
            frameReady = false; // Reset the frame flag
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            stopFlag = true;
            break;
        }
    }

    // Clean up
    if (ffmpegProcess) {
        fclose(ffmpegProcess);
    }

    cv::destroyAllWindows();
    readThread.join();
    zmqThread.join();

    return 0;
}