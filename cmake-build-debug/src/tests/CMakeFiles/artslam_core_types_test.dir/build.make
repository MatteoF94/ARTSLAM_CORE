# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/matteo/IDEs/clion-2021.2.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/matteo/IDEs/clion-2021.2.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/matteo/catkin_ws/src/artslam_core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug

# Include any dependencies generated for this target.
include src/tests/CMakeFiles/artslam_core_types_test.dir/depend.make
# Include the progress variables for this target.
include src/tests/CMakeFiles/artslam_core_types_test.dir/progress.make

# Include the compile flags for this target's objects.
include src/tests/CMakeFiles/artslam_core_types_test.dir/flags.make

src/tests/CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.o: src/tests/CMakeFiles/artslam_core_types_test.dir/flags.make
src/tests/CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.o: ../src/tests/artslam_core_types_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/tests/CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.o"
	cd /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.o -c /home/matteo/catkin_ws/src/artslam_core/src/tests/artslam_core_types_test.cpp

src/tests/CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.i"
	cd /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matteo/catkin_ws/src/artslam_core/src/tests/artslam_core_types_test.cpp > CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.i

src/tests/CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.s"
	cd /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matteo/catkin_ws/src/artslam_core/src/tests/artslam_core_types_test.cpp -o CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.s

# Object files for target artslam_core_types_test
artslam_core_types_test_OBJECTS = \
"CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.o"

# External object files for target artslam_core_types_test
artslam_core_types_test_EXTERNAL_OBJECTS =

devel/lib/artslam_core/artslam_core_types_test: src/tests/CMakeFiles/artslam_core_types_test.dir/artslam_core_types_test.cpp.o
devel/lib/artslam_core/artslam_core_types_test: src/tests/CMakeFiles/artslam_core_types_test.dir/build.make
devel/lib/artslam_core/artslam_core_types_test: devel/lib/libartslam_core_types_library.so
devel/lib/artslam_core/artslam_core_types_test: src/tests/CMakeFiles/artslam_core_types_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../devel/lib/artslam_core/artslam_core_types_test"
	cd /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/artslam_core_types_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tests/CMakeFiles/artslam_core_types_test.dir/build: devel/lib/artslam_core/artslam_core_types_test
.PHONY : src/tests/CMakeFiles/artslam_core_types_test.dir/build

src/tests/CMakeFiles/artslam_core_types_test.dir/clean:
	cd /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests && $(CMAKE_COMMAND) -P CMakeFiles/artslam_core_types_test.dir/cmake_clean.cmake
.PHONY : src/tests/CMakeFiles/artslam_core_types_test.dir/clean

src/tests/CMakeFiles/artslam_core_types_test.dir/depend:
	cd /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matteo/catkin_ws/src/artslam_core /home/matteo/catkin_ws/src/artslam_core/src/tests /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests /home/matteo/catkin_ws/src/artslam_core/cmake-build-debug/src/tests/CMakeFiles/artslam_core_types_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tests/CMakeFiles/artslam_core_types_test.dir/depend
