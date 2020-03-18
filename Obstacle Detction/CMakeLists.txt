
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(test)

set(OpenCV_DIR "/home/songsong/Documents/Softwares/opencv-4.0.1/build")

FIND_PACKAGE( OpenCV REQUIRED )

FIND_PACKAGE( realsense2 REQUIRED )
# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


#set(CMAKE_CXX_STANDARD 11)
SET( CMAKE_BUILD_TYPE Debug  )
#SET(CMAKE_CXX_FLAGS -std=c++11)


ADD_LIBRARY( align_depth_color align_depth_color.cpp)
TARGET_LINK_LIBRARIES( align_depth_color
        ${OpenCV_LIBS})

ADD_LIBRARY(read_images readImages.cpp)

ADD_LIBRARY(inpainting inpainting.cpp)
TARGET_LINK_LIBRARIES( inpainting
        ${OpenCV_LIBS})

add_library(kalman kalman.cpp)
TARGET_LINK_LIBRARIES( kalman
        ${OpenCV_LIBS})

add_library(SSIM SSIM.cpp )
target_link_libraries(SSIM
        ${OpenCV_LIBS})

add_library(hash Hash.cpp)
TARGET_LINK_LIBRARIES( hash
        ${OpenCV_LIBS})


#ADD_LIBRARY(opticalflow opticalflow.cpp )
#target_link_libraries(opticalflow
#        ${OpenCV_LIBS}
#        )


ADD_LIBRARY( contours_process contours_processor.cpp )
TARGET_LINK_LIBRARIES( contours_process
        kalman
        SSIM
        hash
        opticalflow
        ${OpenCV_LIBS})












add_executable(test main.cpp )
target_link_libraries(test
        align_depth_color
        contours_process
        ${DEPENDENCIES}
        librealsense2.so
        ${OpenCV_LIBS}
        )

add_executable(test2 main2.cpp )
target_link_libraries(test2
        read_images
        kalman
        hash
        SSIM
        contours_process
        align_depth_color
        ${DEPENDENCIES}
        ${OpenCV_LIBS}
        )



add_executable(test3 main3.cpp )
target_link_libraries(test3
        hash
        ${DEPENDENCIES}
        ${OpenCV_LIBS}
        )


add_executable(sgm sgm.cpp )
target_link_libraries(sgm
        align_depth_color
        librealsense2.so
        ${OpenCV_LIBS}
        )



add_executable(otsu otsu.cpp )
target_link_libraries(otsu
        ${DEPENDENCIES}
        ${OpenCV_LIBS}
        )

add_executable(phash phash.cpp )
target_link_libraries(phash
        ${OpenCV_LIBS}
        )

add_executable(camera opencamera.cpp)
target_link_libraries(camera
        ${DEPENDENCIES}
        align_depth_color
        librealsense2.so
        ${OpenCV_LIBS}
        )

add_executable(test4 main4.cpp )
target_link_libraries(test4
        inpainting
        ${OpenCV_LIBS}
        )

add_executable(object_detection object_detection.cpp )
target_link_libraries(object_detection
        ${OpenCV_LIBS}
        )


add_executable(pcd pcd.cpp )
target_link_libraries(pcd
        read_images
        ${OpenCV_LIBS}
        )

add_executable(opticalflow opticalflow.cpp )
target_link_libraries(opticalflow
        ${OpenCV_LIBS}
        )

add_executable(video video.cpp )
target_link_libraries(video
        ${OpenCV_LIBS}
        )



#add_executable(SSIM SSIM.cpp )
#target_link_libraries(SSIM
#        ${OpenCV_LIBS}
#        )


#set_target_properties (test PROPERTIES FOLDER "readVideoRealsense")

