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
CMAKE_COMMAND = /snap/clion/70/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/70/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/songsong/Documents/private/Graduation-design/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug

# Include any dependencies generated for this target.
include src/CMakeFiles/generate_pointcloud.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/generate_pointcloud.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/generate_pointcloud.dir/flags.make

src/CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.o: src/CMakeFiles/generate_pointcloud.dir/flags.make
src/CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.o: ../src/generatePointCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.o"
	cd /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.o -c /home/songsong/Documents/private/Graduation-design/test/src/generatePointCloud.cpp

src/CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.i"
	cd /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/private/Graduation-design/test/src/generatePointCloud.cpp > CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.i

src/CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.s"
	cd /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/private/Graduation-design/test/src/generatePointCloud.cpp -o CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.s

# Object files for target generate_pointcloud
generate_pointcloud_OBJECTS = \
"CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.o"

# External object files for target generate_pointcloud
generate_pointcloud_EXTERNAL_OBJECTS =

../bin/generate_pointcloud: src/CMakeFiles/generate_pointcloud.dir/generatePointCloud.cpp.o
../bin/generate_pointcloud: src/CMakeFiles/generate_pointcloud.dir/build.make
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_gapi.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_stitching.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_xobjdetect.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_superres.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_dpm.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_hfs.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_aruco.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_tracking.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_bgsegm.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_rgbd.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_dnn_objdetect.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_line_descriptor.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_face.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_ccalib.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_surface_matching.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_img_hash.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_stereo.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_optflow.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_reg.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_xfeatures2d.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_fuzzy.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_videostab.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_structured_light.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_saliency.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_freetype.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_xphoto.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_plot.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_bioinspired.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_shape.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_ximgproc.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_datasets.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_text.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_ml.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_dnn.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_objdetect.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_video.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_calib3d.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_phase_unwrapping.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_features2d.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_flann.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_photo.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_highgui.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_videoio.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_imgcodecs.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_imgproc.so.4.0.1
../bin/generate_pointcloud: /usr/local/opencv4.0.1/lib/libopencv_core.so.4.0.1
../bin/generate_pointcloud: src/CMakeFiles/generate_pointcloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/generate_pointcloud"
	cd /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/generate_pointcloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/generate_pointcloud.dir/build: ../bin/generate_pointcloud

.PHONY : src/CMakeFiles/generate_pointcloud.dir/build

src/CMakeFiles/generate_pointcloud.dir/clean:
	cd /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/generate_pointcloud.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/generate_pointcloud.dir/clean

src/CMakeFiles/generate_pointcloud.dir/depend:
	cd /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/private/Graduation-design/test /home/songsong/Documents/private/Graduation-design/test/src /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src /home/songsong/Documents/private/Graduation-design/test/cmake-build-debug/src/CMakeFiles/generate_pointcloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/generate_pointcloud.dir/depend

