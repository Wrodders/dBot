#include <opencv2/core.hpp>
#include <iostream>

int main () {
    // Read an image from file
    cv::Mat originalImage = cv::imread("path/to/your/image.jpg");

    // Check if the image was loaded successfully
    if (originalImage.empty()) {
        std::cerr << "Error: Could not read the image." << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat grayscaleImage;
    cv::cvtColor(originalImage, grayscaleImage, cv::COLOR_BGR2GRAY);

    // Display the original and grayscale images
    cv::imshow("Original Image", originalImage);
    cv::imshow("Grayscale Image", grayscaleImage);

    // Wait for a key press and then close the windows
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
