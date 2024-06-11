# Team Members

Name: Vishaq Jayakumar
NUID: 002737793

# Video URL

https://drive.google.com/drive/folders/11x8JygBB5lLehsK8o_NAv1bBAXcKHdpU?usp=drive_link

# Code Description

CameraCalib.cpp - THis code captures frames from the webcam, detects chessboard corners in each frame, and saves the detected corner coordinates along with corresponding world coordinates. After collecting a sufficient number of calibration frames, it calculates the camera matrix and distortion coefficients using OpenCV's camera calibration functions. The calculated intrinsic parameters are then saved to a YAML file named "intrinsics.yml" for later use.

LineDetect.cpp - This code loads camera calibration parameters, captures video from a webcam, detects corners of a specified chessboard pattern size in each frame, refines corner locations, estimates the pose of the pattern, and visualizes the 3D coordinate axes and reprojected corners on the frame.

VirtObjDetect.cpp -  This code first loads camera calibration parameters from a file. Then, it captures video from a webcam, detects corners of a predefined chessboard pattern size in each frame, and refines the corner locations. After that, it estimates the pose of the pattern using the solvePnP function. Finally, it visualizes the 3D coordinate axes and reprojected corners on the frame.

HarrisFeatures.cpp - The program continuously captures frames from the webcam, detects Harris corners in each frame, and displays the frames with detected corners until the user exits by pressing the 'q' key.

# Operating System and IDE

MacOS

Visual Studio Code

# Running the program

CameraCalib.cpp - Press 's' to save the detected corner points or 'q' to exit the corner selection process.  After selecting the corner points from multiple frames, the program will perform camera calibration automatically. The intrinsic parameters will be saved to a YAML file named intrinsics.yml.

LineDetect.cpp - When the program runs, it will attempt to access your webcam. Make sure your webcam is connected and accessible. Press 'q' on your keyboard to exit the program.

VirtObjDetect.cpp - When the program runs, it will attempt to access your webcam. Make sure your webcam is connected and accessible. Press 'q' on your keyboard to exit the program.

HarrisFeatures.cpp - When the program runs, it will attempt to access your webcam. Make sure your webcam is connected and accessible. Press 'q' on your keyboard to exit the program.

Extensions - 
1. Used multiple targets in the scene - VirtObjDetect.cpp
2. Tested out on different cameras as shown in the video.


# Time Travel Days

Three days taken
