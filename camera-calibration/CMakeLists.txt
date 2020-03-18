cmake_minimum_required(VERSION 3.6)
project(camera_calibrate)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(OpenCV_DIR "/home/songsong/Documents/Softwares/opencv-4.0.1/build")

FIND_PACKAGE( OpenCV REQUIRED )

ADD_LIBRARY( calibration calibration.cpp )
TARGET_LINK_LIBRARIES( calibration
        ${OpenCV_LIBS})


ADD_LIBRARY( undistort undistort.cpp )
TARGET_LINK_LIBRARIES( undistort
        ${OpenCV_LIBS})

ADD_LIBRARY( outermatch outermatch.cpp )
TARGET_LINK_LIBRARIES( outermatch
        ${OpenCV_LIBS})


add_executable(calibrate main.cpp)
TARGET_LINK_LIBRARIES( calibrate
        calibration
        undistort
        outermatch
        ${OpenCV_LIBS})