

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")


project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for project (Solution)
add_executable (2D_feature_tracking src/matching2D_Student.cpp src/MidTermProject_Camera_Student.cpp)
target_link_libraries (2D_feature_tracking ${OpenCV_LIBRARIES})

# Executable for project Preformance Evaluation (Keypoints Counter)

add_executable (2D_feature_tracking_keypoints_Counter src/matching2D_Student.cpp src/MidTermProject_Camera_Student_Keypoints_Counter.cpp)
target_link_libraries (2D_feature_tracking_keypoints_Counter ${OpenCV_LIBRARIES})

# # Executable for project Preformance Evaluation (Detectors Descriptors Keypoints Matcher)

# add_executable (2D_feature_tracking_Detectors_Descriptors_Matcher src/matching2D_Student.cpp src/MidTermProject_Camera_Student_Detectors_Descriptors_Matcher.cpp)
# target_link_libraries (2D_feature_tracking_Detectors_Descriptors_Matcher ${OpenCV_LIBRARIES})



