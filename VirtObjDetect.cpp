#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

// Function to generate world points for the chessboard pattern
std::vector<cv::Point3f> generateWorldPoints(const cv::Size& patternSize) {
    std::vector<cv::Point3f> points;
    for (int i = 0; i < patternSize.height; ++i) {
        for (int j = 0; j < patternSize.width; ++j) {
            points.push_back(cv::Point3f(j, -i, 0)); // Assuming Z-axis comes towards the viewer
        }
    }
    return points;
}

// Function to draw a centered pyramid with more height
void draw3dObject(cv::Mat& src, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients, cv::Mat& rvec, cv::Mat& tvec) {
    // Define pyramid points in world space
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Vec3f({2, -2, 3})); // center
    objectPoints.push_back(cv::Vec3f({3, -1, 0})); // tr
    objectPoints.push_back(cv::Vec3f({3, -3, 0})); // br
    objectPoints.push_back(cv::Vec3f({1, -3, 0})); // bl
    objectPoints.push_back(cv::Vec3f({1, -1, 0})); // tl
    
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

    cv::line(src, imagePoints[0], imagePoints[1], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[0], imagePoints[3], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[0], imagePoints[4], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[1], imagePoints[2], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[2], imagePoints[3], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[3], imagePoints[4], cv::Scalar(0, 255, 255), 5);
    cv::line(src, imagePoints[4], imagePoints[1], cv::Scalar(0, 255, 255), 5);
        
    objectPoints.clear();
    imagePoints.clear();
    
    // CUBE
    objectPoints.push_back(cv::Vec3f({8, -5, 0})); // br
    objectPoints.push_back(cv::Vec3f({8, -5, 2}));
    objectPoints.push_back(cv::Vec3f({6, -5, 0})); // bl
    objectPoints.push_back(cv::Vec3f({6, -5, 2}));
    objectPoints.push_back(cv::Vec3f({8, -3, 0})); // tr
    objectPoints.push_back(cv::Vec3f({8, -3, 2}));
    objectPoints.push_back(cv::Vec3f({6, -3, 0})); // tl
    objectPoints.push_back(cv::Vec3f({6, -3, 2}));
    
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

    cv::line(src, imagePoints[0], imagePoints[2], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[4], imagePoints[6], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[0], imagePoints[4], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[2], imagePoints[6], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[1], imagePoints[3], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[5], imagePoints[7], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[1], imagePoints[5], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[3], imagePoints[7], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[0], imagePoints[1], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[2], imagePoints[3], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[4], imagePoints[5], cv::Scalar(255, 255, 0), 5);
    cv::line(src, imagePoints[6], imagePoints[7], cv::Scalar(255, 255, 0), 5);
    
    objectPoints.clear();
    imagePoints.clear();
    
    // PRISM
    objectPoints.push_back(cv::Vec3f({6, -1, 2})); // br
    objectPoints.push_back(cv::Vec3f({4, -1, 2})); // bl
    objectPoints.push_back(cv::Vec3f({6, 0, 2})); // tr
    objectPoints.push_back(cv::Vec3f({4, 0, 2})); // tl
    objectPoints.push_back(cv::Vec3f({6, -1, 4}));
    objectPoints.push_back(cv::Vec3f({6, 0, 4}));
    
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);

    cv::line(src, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[1], imagePoints[3], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[0], imagePoints[2], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[2], imagePoints[5], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[1], imagePoints[4], cv::Scalar(255, 0, 255), 5);
    cv::line(src, imagePoints[3], imagePoints[5], cv::Scalar(255, 0, 255), 5);

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
    while (true) {
        // Read frame from the camera
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

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

            // Draw 3D object (centered pyramid with more height)
            draw3dObject(frame, cameraMatrix, distortionCoefficients, rvec, tvec);
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
