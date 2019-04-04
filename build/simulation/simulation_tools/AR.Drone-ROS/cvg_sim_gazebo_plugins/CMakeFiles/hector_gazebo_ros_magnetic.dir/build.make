# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build

# Include any dependencies generated for this target.
include simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/depend.make

# Include the progress variables for this target.
include simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/progress.make

# Include the compile flags for this target's objects.
include simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/flags.make

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/flags.make
simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o: /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/src/gazebo_ros_magnetic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o"
	cd /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o -c /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/src/gazebo_ros_magnetic.cpp

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.i"
	cd /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/src/gazebo_ros_magnetic.cpp > CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.i

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.s"
	cd /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/src/gazebo_ros_magnetic.cpp -o CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.s

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.requires:

.PHONY : simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.requires

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.provides: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.requires
	$(MAKE) -f simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/build.make simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.provides.build
.PHONY : simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.provides

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.provides.build: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o


# Object files for target hector_gazebo_ros_magnetic
hector_gazebo_ros_magnetic_OBJECTS = \
"CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o"

# External object files for target hector_gazebo_ros_magnetic
hector_gazebo_ros_magnetic_EXTERNAL_OBJECTS =

/home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/libhector_gazebo_ros_magnetic.so: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o
/home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/libhector_gazebo_ros_magnetic.so: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/build.make
/home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/libhector_gazebo_ros_magnetic.so: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/libhector_gazebo_ros_magnetic.so"
	cd /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hector_gazebo_ros_magnetic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/build: /home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/libhector_gazebo_ros_magnetic.so

.PHONY : simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/build

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/requires: simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/src/gazebo_ros_magnetic.cpp.o.requires

.PHONY : simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/requires

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/clean:
	cd /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins && $(CMAKE_COMMAND) -P CMakeFiles/hector_gazebo_ros_magnetic.dir/cmake_clean.cmake
.PHONY : simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/clean

simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/depend:
	cd /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins /home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : simulation/simulation_tools/AR.Drone-ROS/cvg_sim_gazebo_plugins/CMakeFiles/hector_gazebo_ros_magnetic.dir/depend

