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
include CMakeFiles/read_images.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/read_images.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/read_images.dir/flags.make

CMakeFiles/read_images.dir/readImages.cpp.o: CMakeFiles/read_images.dir/flags.make
CMakeFiles/read_images.dir/readImages.cpp.o: ../readImages.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/read_images.dir/readImages.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/read_images.dir/readImages.cpp.o -c /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/readImages.cpp

CMakeFiles/read_images.dir/readImages.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/read_images.dir/readImages.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/readImages.cpp > CMakeFiles/read_images.dir/readImages.cpp.i

CMakeFiles/read_images.dir/readImages.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/read_images.dir/readImages.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/readImages.cpp -o CMakeFiles/read_images.dir/readImages.cpp.s

# Object files for target read_images
read_images_OBJECTS = \
"CMakeFiles/read_images.dir/readImages.cpp.o"

# External object files for target read_images
read_images_EXTERNAL_OBJECTS =

libread_images.a: CMakeFiles/read_images.dir/readImages.cpp.o
libread_images.a: CMakeFiles/read_images.dir/build.make
libread_images.a: CMakeFiles/read_images.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libread_images.a"
	$(CMAKE_COMMAND) -P CMakeFiles/read_images.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/read_images.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/read_images.dir/build: libread_images.a

.PHONY : CMakeFiles/read_images.dir/build

CMakeFiles/read_images.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/read_images.dir/cmake_clean.cmake
.PHONY : CMakeFiles/read_images.dir/clean

CMakeFiles/read_images.dir/depend:
	cd /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1 /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug /home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/cmake-build-debug/CMakeFiles/read_images.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/read_images.dir/depend
