#include <iostream>
#include <opencv2/opencv.hpp>

int detectHarrisCorners(cv::Mat &src, cv::Mat &dst) {
    int thresh = 150;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    
    dst = src.clone();
    cv::Mat gray, tmp, tmp_norm;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    tmp = cv::Mat::zeros(src.size(), CV_32FC1);

    // Detecting corners
    cv::cornerHarris(gray, tmp, blockSize, apertureSize, k);

    // Normalizing
    cv::normalize(tmp, tmp_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // Drawing a circle around corners
    for(int j = 0; j < tmp_norm.rows ; j++) {
        for(int i = 0; i < tmp_norm.cols; i++) {
            if((int) tmp_norm.at<float>(j,i) > thresh) {
                cv::circle(dst, cv::Point(i, j), 2, cv::Scalar(0, 0, 255), 2, 8, 0 );
            }
        }
    }
        
    return 0;
}

int main() {
    // Start video capture
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Failed to open webcam." << std::endl;
        return -1;
    }

    cv::Mat frame, frame_with_corners;

    while (true) {
        // Read frame from the camera
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

        // Detect Harris corners
        detectHarrisCorners(frame, frame_with_corners);

        // Show the frame with Harris corners
        cv::imshow("Frame with Harris Corners", frame_with_corners);

        // Check for 'q' key to exit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
