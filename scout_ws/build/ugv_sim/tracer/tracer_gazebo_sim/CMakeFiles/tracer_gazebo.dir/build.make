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
CMAKE_SOURCE_DIR = /home/hugoc/scout_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hugoc/scout_ws/build

# Include any dependencies generated for this target.
include ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/depend.make

# Include the progress variables for this target.
include ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/progress.make

# Include the compile flags for this target's objects.
include ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/flags.make

ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.o: ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/flags.make
ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.o: /home/hugoc/scout_ws/src/ugv_sim/tracer/tracer_gazebo_sim/src/tracer_skid_steer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hugoc/scout_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.o"
	cd /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.o -c /home/hugoc/scout_ws/src/ugv_sim/tracer/tracer_gazebo_sim/src/tracer_skid_steer.cpp

ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.i"
	cd /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hugoc/scout_ws/src/ugv_sim/tracer/tracer_gazebo_sim/src/tracer_skid_steer.cpp > CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.i

ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.s"
	cd /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hugoc/scout_ws/src/ugv_sim/tracer/tracer_gazebo_sim/src/tracer_skid_steer.cpp -o CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.s

# Object files for target tracer_gazebo
tracer_gazebo_OBJECTS = \
"CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.o"

# External object files for target tracer_gazebo
tracer_gazebo_EXTERNAL_OBJECTS =

/home/hugoc/scout_ws/devel/lib/libtracer_gazebo.a: ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/src/tracer_skid_steer.cpp.o
/home/hugoc/scout_ws/devel/lib/libtracer_gazebo.a: ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/build.make
/home/hugoc/scout_ws/devel/lib/libtracer_gazebo.a: ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hugoc/scout_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/hugoc/scout_ws/devel/lib/libtracer_gazebo.a"
	cd /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim && $(CMAKE_COMMAND) -P CMakeFiles/tracer_gazebo.dir/cmake_clean_target.cmake
	cd /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracer_gazebo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/build: /home/hugoc/scout_ws/devel/lib/libtracer_gazebo.a

.PHONY : ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/build

ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/clean:
	cd /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim && $(CMAKE_COMMAND) -P CMakeFiles/tracer_gazebo.dir/cmake_clean.cmake
.PHONY : ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/clean

ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/depend:
	cd /home/hugoc/scout_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hugoc/scout_ws/src /home/hugoc/scout_ws/src/ugv_sim/tracer/tracer_gazebo_sim /home/hugoc/scout_ws/build /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim /home/hugoc/scout_ws/build/ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ugv_sim/tracer/tracer_gazebo_sim/CMakeFiles/tracer_gazebo.dir/depend

