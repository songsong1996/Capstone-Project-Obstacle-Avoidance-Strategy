cmake_minimum_required(VERSION 2.8)
project(camera_opencv)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


find_package(catkin REQUIRED COMPONENTS 
cv_bridge 
image_transport 
roscpp 
rospy 
std_msgs 
sensor_msgs 
message_generation 
realsense2_camera
)

find_package(OpenCV REQUIRED)
FIND_PACKAGE( realsense2 REQUIRED )

# add the resized image message
add_message_files(DIRECTORY msg
   FILES get_top_view.msg
)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_opencv
CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs sensor_msgs message_runtime 
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


ADD_LIBRARY( align_depth_color src/align_depth_color.cpp)
TARGET_LINK_LIBRARIES( align_depth_color
        ${OpenCV_LIBS})

ADD_LIBRARY(kalman src/kalman.cpp)
TARGET_LINK_LIBRARIES( kalman
        ${OpenCV_LIBS})



ADD_LIBRARY(hash src/Hash.cpp)
TARGET_LINK_LIBRARIES( hash
        ${OpenCV_LIBS})

ADD_LIBRARY( contours_process src/contours_processor.cpp )
TARGET_LINK_LIBRARIES( contours_process
        kalman
        hash
        ${OpenCV_LIBS})


# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
add_dependencies(my_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the top_view_image
add_executable(rosopencv src/rosopencv.cpp)
target_link_libraries(
	rosopencv 
        kalman
        hash
        contours_process
        align_depth_color 
	librealsense2.so 
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES})


