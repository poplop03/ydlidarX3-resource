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
include examples/CMakeFiles/et_test.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/et_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/et_test.dir/flags.make

examples/CMakeFiles/et_test.dir/et_test.cpp.o: examples/CMakeFiles/et_test.dir/flags.make
examples/CMakeFiles/et_test.dir/et_test.cpp.o: ../examples/et_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/et_test.dir/et_test.cpp.o"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/et_test.dir/et_test.cpp.o -c /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/et_test.cpp

examples/CMakeFiles/et_test.dir/et_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/et_test.dir/et_test.cpp.i"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/et_test.cpp > CMakeFiles/et_test.dir/et_test.cpp.i

examples/CMakeFiles/et_test.dir/et_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/et_test.dir/et_test.cpp.s"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples/et_test.cpp -o CMakeFiles/et_test.dir/et_test.cpp.s

# Object files for target et_test
et_test_OBJECTS = \
"CMakeFiles/et_test.dir/et_test.cpp.o"

# External object files for target et_test
et_test_EXTERNAL_OBJECTS =

et_test: examples/CMakeFiles/et_test.dir/et_test.cpp.o
et_test: examples/CMakeFiles/et_test.dir/build.make
et_test: libydlidar_sdk.a
et_test: examples/CMakeFiles/et_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../et_test"
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/et_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/et_test.dir/build: et_test

.PHONY : examples/CMakeFiles/et_test.dir/build

examples/CMakeFiles/et_test.dir/clean:
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/et_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/et_test.dir/clean

examples/CMakeFiles/et_test.dir/depend:
	cd /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/examples /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples /home/pc/ydlidarX3-resource/RDK_Code/YDLidar-SDK-master/build/examples/CMakeFiles/et_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/et_test.dir/depend
