# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/lidar_c_api_test.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/lidar_c_api_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/lidar_c_api_test.dir/flags.make

examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o: examples/CMakeFiles/lidar_c_api_test.dir/flags.make
examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o: ../examples/lidar_c_api_test.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o   -c /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/lidar_c_api_test.c

examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.i"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/lidar_c_api_test.c > CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.i

examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.s"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/lidar_c_api_test.c -o CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.s

# Object files for target lidar_c_api_test
lidar_c_api_test_OBJECTS = \
"CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o"

# External object files for target lidar_c_api_test
lidar_c_api_test_EXTERNAL_OBJECTS =

lidar_c_api_test: examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o
lidar_c_api_test: examples/CMakeFiles/lidar_c_api_test.dir/build.make
lidar_c_api_test: libydlidar_sdk.a
lidar_c_api_test: examples/CMakeFiles/lidar_c_api_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../lidar_c_api_test"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar_c_api_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/lidar_c_api_test.dir/build: lidar_c_api_test

.PHONY : examples/CMakeFiles/lidar_c_api_test.dir/build

examples/CMakeFiles/lidar_c_api_test.dir/clean:
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/lidar_c_api_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/lidar_c_api_test.dir/clean

examples/CMakeFiles/lidar_c_api_test.dir/depend:
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples/CMakeFiles/lidar_c_api_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/lidar_c_api_test.dir/depend

