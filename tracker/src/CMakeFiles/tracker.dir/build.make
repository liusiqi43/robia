# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/me/Documents/robia/tracker/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/me/Documents/robia/tracker/src

# Include any dependencies generated for this target.
include CMakeFiles/tracker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tracker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tracker.dir/flags.make

CMakeFiles/tracker.dir/dyeFilter.cc.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/dyeFilter.cc.o: dyeFilter.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/me/Documents/robia/tracker/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracker.dir/dyeFilter.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/dyeFilter.cc.o -c /home/me/Documents/robia/tracker/src/dyeFilter.cc

CMakeFiles/tracker.dir/dyeFilter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/dyeFilter.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/me/Documents/robia/tracker/src/dyeFilter.cc > CMakeFiles/tracker.dir/dyeFilter.cc.i

CMakeFiles/tracker.dir/dyeFilter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/dyeFilter.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/me/Documents/robia/tracker/src/dyeFilter.cc -o CMakeFiles/tracker.dir/dyeFilter.cc.s

CMakeFiles/tracker.dir/dyeFilter.cc.o.requires:
.PHONY : CMakeFiles/tracker.dir/dyeFilter.cc.o.requires

CMakeFiles/tracker.dir/dyeFilter.cc.o.provides: CMakeFiles/tracker.dir/dyeFilter.cc.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/dyeFilter.cc.o.provides.build
.PHONY : CMakeFiles/tracker.dir/dyeFilter.cc.o.provides

CMakeFiles/tracker.dir/dyeFilter.cc.o.provides.build: CMakeFiles/tracker.dir/dyeFilter.cc.o

CMakeFiles/tracker.dir/main.cc.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/main.cc.o: main.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/me/Documents/robia/tracker/src/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracker.dir/main.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/main.cc.o -c /home/me/Documents/robia/tracker/src/main.cc

CMakeFiles/tracker.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/main.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/me/Documents/robia/tracker/src/main.cc > CMakeFiles/tracker.dir/main.cc.i

CMakeFiles/tracker.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/main.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/me/Documents/robia/tracker/src/main.cc -o CMakeFiles/tracker.dir/main.cc.s

CMakeFiles/tracker.dir/main.cc.o.requires:
.PHONY : CMakeFiles/tracker.dir/main.cc.o.requires

CMakeFiles/tracker.dir/main.cc.o.provides: CMakeFiles/tracker.dir/main.cc.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/main.cc.o.provides.build
.PHONY : CMakeFiles/tracker.dir/main.cc.o.provides

CMakeFiles/tracker.dir/main.cc.o.provides.build: CMakeFiles/tracker.dir/main.cc.o

# Object files for target tracker
tracker_OBJECTS = \
"CMakeFiles/tracker.dir/dyeFilter.cc.o" \
"CMakeFiles/tracker.dir/main.cc.o"

# External object files for target tracker
tracker_EXTERNAL_OBJECTS =

tracker: CMakeFiles/tracker.dir/dyeFilter.cc.o
tracker: CMakeFiles/tracker.dir/main.cc.o
tracker: CMakeFiles/tracker.dir/build.make
tracker: /usr/local/lib/libopencv_calib3d.so
tracker: /usr/local/lib/libopencv_contrib.so
tracker: /usr/local/lib/libopencv_core.so
tracker: /usr/local/lib/libopencv_features2d.so
tracker: /usr/local/lib/libopencv_flann.so
tracker: /usr/local/lib/libopencv_gpu.so
tracker: /usr/local/lib/libopencv_highgui.so
tracker: /usr/local/lib/libopencv_imgproc.so
tracker: /usr/local/lib/libopencv_legacy.so
tracker: /usr/local/lib/libopencv_ml.so
tracker: /usr/local/lib/libopencv_nonfree.so
tracker: /usr/local/lib/libopencv_objdetect.so
tracker: /usr/local/lib/libopencv_photo.so
tracker: /usr/local/lib/libopencv_stitching.so
tracker: /usr/local/lib/libopencv_ts.so
tracker: /usr/local/lib/libopencv_video.so
tracker: /usr/local/lib/libopencv_videostab.so
tracker: CMakeFiles/tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable tracker"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tracker.dir/build: tracker
.PHONY : CMakeFiles/tracker.dir/build

CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/dyeFilter.cc.o.requires
CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/main.cc.o.requires
.PHONY : CMakeFiles/tracker.dir/requires

CMakeFiles/tracker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tracker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tracker.dir/clean

CMakeFiles/tracker.dir/depend:
	cd /home/me/Documents/robia/tracker/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/me/Documents/robia/tracker/src /home/me/Documents/robia/tracker/src /home/me/Documents/robia/tracker/src /home/me/Documents/robia/tracker/src /home/me/Documents/robia/tracker/src/CMakeFiles/tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tracker.dir/depend
