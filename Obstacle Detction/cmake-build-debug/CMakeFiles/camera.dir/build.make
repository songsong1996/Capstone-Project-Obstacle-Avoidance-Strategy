# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/73/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/73/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera.dir/flags.make

CMakeFiles/camera.dir/opencamera.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/opencamera.cpp.o: ../opencamera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera.dir/opencamera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/opencamera.cpp.o -c /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/opencamera.cpp

CMakeFiles/camera.dir/opencamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/opencamera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/opencamera.cpp > CMakeFiles/camera.dir/opencamera.cpp.i

CMakeFiles/camera.dir/opencamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/opencamera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/opencamera.cpp -o CMakeFiles/camera.dir/opencamera.cpp.s

# Object files for target camera
camera_OBJECTS = \
"CMakeFiles/camera.dir/opencamera.cpp.o"

# External object files for target camera
camera_EXTERNAL_OBJECTS =

camera: CMakeFiles/camera.dir/opencamera.cpp.o
camera: CMakeFiles/camera.dir/build.make
camera: libalign_depth_color.a
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_gapi.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_stitching.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_xobjdetect.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_superres.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_dpm.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_hfs.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_aruco.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_tracking.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_bgsegm.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_rgbd.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_dnn_objdetect.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_line_descriptor.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_face.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_ccalib.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_surface_matching.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_img_hash.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_stereo.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_optflow.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_reg.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_xfeatures2d.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_fuzzy.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_videostab.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_structured_light.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_saliency.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_freetype.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_xphoto.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_plot.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_bioinspired.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_datasets.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_text.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_dnn.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_objdetect.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_ximgproc.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_ml.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_shape.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_video.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_calib3d.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_phase_unwrapping.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_features2d.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_flann.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_photo.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_highgui.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_videoio.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_imgcodecs.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_imgproc.so.4.0.1
camera: /home/songsong/Documents/Softwares/opencv-4.0.1/build/lib/libopencv_core.so.4.0.1
camera: CMakeFiles/camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable camera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera.dir/build: camera

.PHONY : CMakeFiles/camera.dir/build

CMakeFiles/camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera.dir/clean

CMakeFiles/camera.dir/depend:
	cd /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles/camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera.dir/depend

