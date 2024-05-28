#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>


std::vector<cv::Point3f> generateWorldPoints(const cv::Size& patternSize) {
    std::vector<cv::Point3f> points;
    for (int i = 0; i < patternSize.height; ++i) {
        for (int j = 0; j < patternSize.width; ++j) {
            points.push_back(cv::Point3f(j, -i, 0)); // Assuming Z-axis comes towards the viewer
        }
    }
    return points;
}


int main() {
    // Load camera calibration parameters
    cv::Mat cameraMatrix, distortionCoefficients;
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Failed to open camera calibration file." << std::endl;
        return -1;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distortionCoefficients;
    fs.release();

    // Start video capture
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Failed to open webcam." << std::endl;
        return -1;
    }

    cv::Size patternSize(9, 6); // Change according to your target pattern

    cv::Mat frame;
    while (cap.read(frame)) {
        // Convert frame to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect corners
        std::vector<cv::Point2f> corners;
        bool patternFound = cv::findChessboardCorners(gray, patternSize, corners);

        if (patternFound) {
            // Refine corner locations
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            // Draw detected corners on the frame
            cv::drawChessboardCorners(frame, patternSize, corners, patternFound);

            // Estimate pose using solvePnP
            cv::Mat rvec, tvec;
            cv::solvePnP(generateWorldPoints(patternSize), corners, cameraMatrix, distortionCoefficients, rvec, tvec);

            // Project 3D points onto the image plane
            std::vector<cv::Point3f> objectPoints;
            objectPoints.push_back(cv::Point3f(0, 0, 0)); // Origin
            objectPoints.push_back(cv::Point3f(3, 0, 0)); // X-axis endpoint
            objectPoints.push_back(cv::Point3f(0, 3, 0)); // Y-axis endpoint
            objectPoints.push_back(cv::Point3f(0, 0, 3)); // Z-axis endpoint
            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

            // Draw axes
            cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X-axis (red)
            cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y-axis (green)
            cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z-axis (blue)

            // Draw reprojected corners
            for (const auto& point : corners) {
                cv::circle(frame, point, 4, cv::Scalar(255, 255, 0), -1); // Yellow
            }
        }

        // Show the frame
        cv::imshow("Frame", frame);

        // Check for 'q' key to exit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
