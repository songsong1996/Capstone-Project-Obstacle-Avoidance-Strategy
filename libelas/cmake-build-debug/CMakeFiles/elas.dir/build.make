# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/songsong/Documents/Softwares/Clion/clion-2018.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/songsong/Documents/Softwares/Clion/clion-2018.3.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/elas.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/elas.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/elas.dir/flags.make

CMakeFiles/elas.dir/src/descriptor.cpp.o: CMakeFiles/elas.dir/flags.make
CMakeFiles/elas.dir/src/descriptor.cpp.o: ../src/descriptor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/elas.dir/src/descriptor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/elas.dir/src/descriptor.cpp.o -c "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/src/descriptor.cpp"

CMakeFiles/elas.dir/src/descriptor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/elas.dir/src/descriptor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/src/descriptor.cpp" > CMakeFiles/elas.dir/src/descriptor.cpp.i

CMakeFiles/elas.dir/src/descriptor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/elas.dir/src/descriptor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/src/descriptor.cpp" -o CMakeFiles/elas.dir/src/descriptor.cpp.s

CMakeFiles/elas.dir/src/main.cpp.o: CMakeFiles/elas.dir/flags.make
CMakeFiles/elas.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/elas.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/elas.dir/src/main.cpp.o -c "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/src/main.cpp"

CMakeFiles/elas.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/elas.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/src/main.cpp" > CMakeFiles/elas.dir/src/main.cpp.i

CMakeFiles/elas.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/elas.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/src/main.cpp" -o CMakeFiles/elas.dir/src/main.cpp.s

# Object files for target elas
elas_OBJECTS = \
"CMakeFiles/elas.dir/src/descriptor.cpp.o" \
"CMakeFiles/elas.dir/src/main.cpp.o"

# External object files for target elas
elas_EXTERNAL_OBJECTS =

elas: CMakeFiles/elas.dir/src/descriptor.cpp.o
elas: CMakeFiles/elas.dir/src/main.cpp.o
elas: CMakeFiles/elas.dir/build.make
elas: CMakeFiles/elas.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable elas"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/elas.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/elas.dir/build: elas

.PHONY : CMakeFiles/elas.dir/build

CMakeFiles/elas.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/elas.dir/cmake_clean.cmake
.PHONY : CMakeFiles/elas.dir/clean

CMakeFiles/elas.dir/depend:
	cd "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)" "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)" "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug" "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug" "/home/songsong/Documents/private/Graduation design/codes/libelas (copy)/cmake-build-debug/CMakeFiles/elas.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/elas.dir/depend

