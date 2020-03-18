

#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(test)

set(OpenCV_DIR "/home/songsong/Documents/Softwares/opencv-4.0.1/build")

FIND_PACKAGE( OpenCV REQUIRED )



include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

add_library(kalman kalman.cpp main3.cpp)
TARGET_LINK_LIBRARIES( kalman
        ${OpenCV_LIBS})

add_executable(test main.cpp )
target_link_libraries(test
        ${OpenCV_LIBS}
        )




add_executable(mouse mouse.cpp )
target_link_libraries(mouse
        ${OpenCV_LIBS}
        )


add_executable(main3 main3.cpp )
target_link_libraries(main3
        kalman
        ${OpenCV_LIBS}
        )


add_executable(main4 main4.cpp )
target_link_libraries(main4
        ${OpenCV_LIBS}
        )


add_executable(kalman2 kalman2.cpp )
target_link_libraries(kalman2
        ${OpenCV_LIBS}
        )
