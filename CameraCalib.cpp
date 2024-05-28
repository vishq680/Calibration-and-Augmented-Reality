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

void detectAndDrawCorners(cv::Mat& image, const cv::Size& patternSize,
                          std::vector<std::vector<cv::Point2f>>& corner_list,
                          std::vector<std::vector<cv::Point3f>>& point_list) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool patternFound = cv::findChessboardCorners(gray, patternSize, corners);

    if (patternFound) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        cv::drawChessboardCorners(image, patternSize, corners, patternFound);

        corner_list.push_back(corners);
        point_list.push_back(generateWorldPoints(patternSize)); // Adjusted to generate Point3f

        std::cout << "Number of corners detected: " << corners.size() << std::endl;
        std::cout << "Coordinates of the first corner: " << corners[0] << std::endl;
    } else {
        std::cerr << "Error: Could not find pattern." << std::endl;
    }
}

void calibrateCamera(const std::vector<std::vector<cv::Point3f>>& objectPointsList,
                     const std::vector<std::vector<cv::Point2f>>& imagePointsList,
                     const cv::Size& imageSize,
                     cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients,
                     std::vector<cv::Mat>& rotations, std::vector<cv::Mat>& translations,
                     double& reprojectionError) {
    std::vector<double> distCoeffs(4, 0.0); // Assuming no radial distortion
    reprojectionError = cv::calibrateCamera(objectPointsList, imagePointsList, imageSize,
                                             cameraMatrix, distortionCoefficients,
                                             rotations, translations, cv::CALIB_FIX_ASPECT_RATIO,
                                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, DBL_EPSILON));
    std::cout << "Reprojection error: " << reprojectionError << std::endl;
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Failed to open webcam." << std::endl;
        return -1;
    }

    cv::Size patternSize(9, 6);
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Point3f>> point_list;

    char key;
    bool saveCorners = true; // Initially set to true to ensure at least one iteration

    std::cout << "Press 's' to save the points or 'q' to exit." << std::endl;

    cv::Mat frame;
    do {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break; // Exit loop if frame capture fails
        }

        detectAndDrawCorners(frame, patternSize, corner_list, point_list);

        cv::imshow("Detected Corners", frame);
        key = cv::waitKey(1);

        if (key == 's') {
            std::cout << "Points saved." << std::endl;
        } else if (key == 'q') {
            saveCorners = false;
            std::cout << "Exiting calibration frame selection." << std::endl;
        }

    } while (saveCorners); // Continue until 'q' is pressed

    if (corner_list.size() < 5) {
        std::cerr << "Error: Insufficient calibration frames selected. Please select at least 5 frames." << std::endl;
        return -1;
    }

    // Calculate the optical center based on the last captured frame
    cv::Size imageSize = frame.size(); // Using the size of the last captured frame

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1; // Focal length in x-direction
    cameraMatrix.at<double>(1, 1) = 1; // Focal length in y-direction
    cameraMatrix.at<double>(0, 2) = imageSize.width / 2; // Optical center x-coordinate
    cameraMatrix.at<double>(1, 2) = imageSize.height / 2; // Optical center y-coordinate
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;


    cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rotations, translations;
    double reprojectionError;

    calibrateCamera(point_list, corner_list, imageSize, cameraMatrix, distortionCoefficients, rotations, translations, reprojectionError);

    std::cout << "Initial Camera Matrix:" << std::endl;
    std::cout << cameraMatrix << std::endl;
    std::cout << "Initial Distortion Coefficients:" << std::endl;
    std::cout << distortionCoefficients << std::endl;

    std::cout << "Final Reprojection Error: " << reprojectionError << std::endl;

    // Save intrinsic parameters to file
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distortionCoefficients;
        fs.release();
    } else {
        std::cerr << "Error: Failed to open file for writing." << std::endl;
    }

    return 0;
}
