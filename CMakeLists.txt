cmake_minimum_required(VERSION 3.10)
project(Project4)
find_package(OpenCV REQUIRED)

add_executable(Project4 VirtObjDetect.cpp)
target_link_libraries(Project4 ${OpenCV_LIBS} ${OPENGL_LIBRARY} ${GLUT_LIBRARY})
# include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
