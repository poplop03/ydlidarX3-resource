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
include examples/CMakeFiles/tri_test.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/tri_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/tri_test.dir/flags.make

examples/CMakeFiles/tri_test.dir/tri_test.cpp.o: examples/CMakeFiles/tri_test.dir/flags.make
examples/CMakeFiles/tri_test.dir/tri_test.cpp.o: ../examples/tri_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/tri_test.dir/tri_test.cpp.o"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tri_test.dir/tri_test.cpp.o -c /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/tri_test.cpp

examples/CMakeFiles/tri_test.dir/tri_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tri_test.dir/tri_test.cpp.i"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/tri_test.cpp > CMakeFiles/tri_test.dir/tri_test.cpp.i

examples/CMakeFiles/tri_test.dir/tri_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tri_test.dir/tri_test.cpp.s"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/tri_test.cpp -o CMakeFiles/tri_test.dir/tri_test.cpp.s

# Object files for target tri_test
tri_test_OBJECTS = \
"CMakeFiles/tri_test.dir/tri_test.cpp.o"

# External object files for target tri_test
tri_test_EXTERNAL_OBJECTS =

tri_test: examples/CMakeFiles/tri_test.dir/tri_test.cpp.o
tri_test: examples/CMakeFiles/tri_test.dir/build.make
tri_test: libydlidar_sdk.a
tri_test: examples/CMakeFiles/tri_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../tri_test"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tri_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/tri_test.dir/build: tri_test

.PHONY : examples/CMakeFiles/tri_test.dir/build

examples/CMakeFiles/tri_test.dir/clean:
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/tri_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/tri_test.dir/clean

examples/CMakeFiles/tri_test.dir/depend:
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples/CMakeFiles/tri_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/tri_test.dir/depend

