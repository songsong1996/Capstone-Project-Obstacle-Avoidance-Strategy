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
include CMakeFiles/align_depth_color.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/align_depth_color.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/align_depth_color.dir/flags.make

CMakeFiles/align_depth_color.dir/align_depth_color.cpp.o: CMakeFiles/align_depth_color.dir/flags.make
CMakeFiles/align_depth_color.dir/align_depth_color.cpp.o: ../align_depth_color.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/align_depth_color.dir/align_depth_color.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/align_depth_color.dir/align_depth_color.cpp.o -c /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/align_depth_color.cpp

CMakeFiles/align_depth_color.dir/align_depth_color.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/align_depth_color.dir/align_depth_color.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/align_depth_color.cpp > CMakeFiles/align_depth_color.dir/align_depth_color.cpp.i

CMakeFiles/align_depth_color.dir/align_depth_color.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/align_depth_color.dir/align_depth_color.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/align_depth_color.cpp -o CMakeFiles/align_depth_color.dir/align_depth_color.cpp.s

# Object files for target align_depth_color
align_depth_color_OBJECTS = \
"CMakeFiles/align_depth_color.dir/align_depth_color.cpp.o"

# External object files for target align_depth_color
align_depth_color_EXTERNAL_OBJECTS =

libalign_depth_color.a: CMakeFiles/align_depth_color.dir/align_depth_color.cpp.o
libalign_depth_color.a: CMakeFiles/align_depth_color.dir/build.make
libalign_depth_color.a: CMakeFiles/align_depth_color.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libalign_depth_color.a"
	$(CMAKE_COMMAND) -P CMakeFiles/align_depth_color.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/align_depth_color.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/align_depth_color.dir/build: libalign_depth_color.a

.PHONY : CMakeFiles/align_depth_color.dir/build

CMakeFiles/align_depth_color.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/align_depth_color.dir/cmake_clean.cmake
.PHONY : CMakeFiles/align_depth_color.dir/clean

CMakeFiles/align_depth_color.dir/depend:
	cd /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles/align_depth_color.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/align_depth_color.dir/depend
