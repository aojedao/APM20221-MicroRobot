# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build

# Include any dependencies generated for this target.
include bbblue_drivers/CMakeFiles/battery_state.dir/depend.make

# Include the progress variables for this target.
include bbblue_drivers/CMakeFiles/battery_state.dir/progress.make

# Include the compile flags for this target's objects.
include bbblue_drivers/CMakeFiles/battery_state.dir/flags.make

bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o: bbblue_drivers/CMakeFiles/battery_state.dir/flags.make
bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o: /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src/bbblue_drivers/src/bb-blue-battery-state.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o"
	cd /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o -c /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src/bbblue_drivers/src/bb-blue-battery-state.cpp

bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.i"
	cd /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src/bbblue_drivers/src/bb-blue-battery-state.cpp > CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.i

bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.s"
	cd /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src/bbblue_drivers/src/bb-blue-battery-state.cpp -o CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.s

bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.requires:

.PHONY : bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.requires

bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.provides: bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.requires
	$(MAKE) -f bbblue_drivers/CMakeFiles/battery_state.dir/build.make bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.provides.build
.PHONY : bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.provides

bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.provides.build: bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o


# Object files for target battery_state
battery_state_OBJECTS = \
"CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o"

# External object files for target battery_state
battery_state_EXTERNAL_OBJECTS =

/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: bbblue_drivers/CMakeFiles/battery_state.dir/build.make
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libtf.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libtf2_ros.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libactionlib.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libmessage_filters.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libroscpp.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libtf2.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/librosconsole.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/librostime.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /opt/ros/melodic/lib/libcpp_common.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so.0.4
/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state: bbblue_drivers/CMakeFiles/battery_state.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state"
	cd /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/battery_state.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bbblue_drivers/CMakeFiles/battery_state.dir/build: /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/devel/lib/bbblue_drivers/battery_state

.PHONY : bbblue_drivers/CMakeFiles/battery_state.dir/build

bbblue_drivers/CMakeFiles/battery_state.dir/requires: bbblue_drivers/CMakeFiles/battery_state.dir/src/bb-blue-battery-state.cpp.o.requires

.PHONY : bbblue_drivers/CMakeFiles/battery_state.dir/requires

bbblue_drivers/CMakeFiles/battery_state.dir/clean:
	cd /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers && $(CMAKE_COMMAND) -P CMakeFiles/battery_state.dir/cmake_clean.cmake
.PHONY : bbblue_drivers/CMakeFiles/battery_state.dir/clean

bbblue_drivers/CMakeFiles/battery_state.dir/depend:
	cd /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/src/bbblue_drivers /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers /home/beagle/UN/APM/APM20221-MicroRobot/catkin_ws/build/bbblue_drivers/CMakeFiles/battery_state.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : bbblue_drivers/CMakeFiles/battery_state.dir/depend

