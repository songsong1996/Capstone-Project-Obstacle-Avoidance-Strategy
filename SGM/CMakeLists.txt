
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


#
#ADD_LIBRARY( SGM SGM.cpp)
#TARGET_LINK_LIBRARIES( SGM
#        ${OpenCV_LIBS})


add_executable(main main.cpp )
target_link_libraries(main
#        SGM
        ${DEPENDENCIES}
        ${OpenCV_LIBS}
        )
