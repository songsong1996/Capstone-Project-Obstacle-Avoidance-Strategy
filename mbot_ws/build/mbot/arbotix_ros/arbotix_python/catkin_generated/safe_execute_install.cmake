execute_process(COMMAND "/home/songsong/catkin_workspace/mbot_ws/build/mbot/arbotix_ros/arbotix_python/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/songsong/catkin_workspace/mbot_ws/build/mbot/arbotix_ros/arbotix_python/catkin_generated/python_distutils_install.sh) returned error code ")
endif()