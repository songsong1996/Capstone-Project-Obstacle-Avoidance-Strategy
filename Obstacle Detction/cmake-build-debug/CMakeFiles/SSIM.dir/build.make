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
include CMakeFiles/SSIM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SSIM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SSIM.dir/flags.make

CMakeFiles/SSIM.dir/SSIM.cpp.o: CMakeFiles/SSIM.dir/flags.make
CMakeFiles/SSIM.dir/SSIM.cpp.o: ../SSIM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SSIM.dir/SSIM.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SSIM.dir/SSIM.cpp.o -c /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/SSIM.cpp

CMakeFiles/SSIM.dir/SSIM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SSIM.dir/SSIM.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/SSIM.cpp > CMakeFiles/SSIM.dir/SSIM.cpp.i

CMakeFiles/SSIM.dir/SSIM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SSIM.dir/SSIM.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/SSIM.cpp -o CMakeFiles/SSIM.dir/SSIM.cpp.s

# Object files for target SSIM
SSIM_OBJECTS = \
"CMakeFiles/SSIM.dir/SSIM.cpp.o"

# External object files for target SSIM
SSIM_EXTERNAL_OBJECTS =

libSSIM.a: CMakeFiles/SSIM.dir/SSIM.cpp.o
libSSIM.a: CMakeFiles/SSIM.dir/build.make
libSSIM.a: CMakeFiles/SSIM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSSIM.a"
	$(CMAKE_COMMAND) -P CMakeFiles/SSIM.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SSIM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SSIM.dir/build: libSSIM.a

.PHONY : CMakeFiles/SSIM.dir/build

CMakeFiles/SSIM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SSIM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SSIM.dir/clean

CMakeFiles/SSIM.dir/depend:
	cd /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles/SSIM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SSIM.dir/depend

