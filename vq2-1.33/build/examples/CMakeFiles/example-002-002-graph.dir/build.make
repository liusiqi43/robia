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
CMAKE_SOURCE_DIR = /home/siqi/Documents/robia/robia/vq2-1.33

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/siqi/Documents/robia/robia/vq2-1.33/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-002-002-graph.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example-002-002-graph.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-002-002-graph.dir/flags.make

examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o: examples/CMakeFiles/example-002-002-graph.dir/flags.make
examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o: ../examples/example-002-002-graph.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siqi/Documents/robia/robia/vq2-1.33/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o"
	cd /home/siqi/Documents/robia/robia/vq2-1.33/build/examples && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -std=c++0x -o CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o -c /home/siqi/Documents/robia/robia/vq2-1.33/examples/example-002-002-graph.cc

examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.i"
	cd /home/siqi/Documents/robia/robia/vq2-1.33/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -std=c++0x -E /home/siqi/Documents/robia/robia/vq2-1.33/examples/example-002-002-graph.cc > CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.i

examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.s"
	cd /home/siqi/Documents/robia/robia/vq2-1.33/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -std=c++0x -S /home/siqi/Documents/robia/robia/vq2-1.33/examples/example-002-002-graph.cc -o CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.s

examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.requires:
.PHONY : examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.requires

examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.provides: examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.requires
	$(MAKE) -f examples/CMakeFiles/example-002-002-graph.dir/build.make examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.provides.build
.PHONY : examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.provides

examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.provides.build: examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o

# Object files for target example-002-002-graph
example__002__002__graph_OBJECTS = \
"CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o"

# External object files for target example-002-002-graph
example__002__002__graph_EXTERNAL_OBJECTS =

examples/example-002-002-graph: examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o
examples/example-002-002-graph: examples/CMakeFiles/example-002-002-graph.dir/build.make
examples/example-002-002-graph: examples/CMakeFiles/example-002-002-graph.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable example-002-002-graph"
	cd /home/siqi/Documents/robia/robia/vq2-1.33/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-002-002-graph.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-002-002-graph.dir/build: examples/example-002-002-graph
.PHONY : examples/CMakeFiles/example-002-002-graph.dir/build

examples/CMakeFiles/example-002-002-graph.dir/requires: examples/CMakeFiles/example-002-002-graph.dir/example-002-002-graph.cc.o.requires
.PHONY : examples/CMakeFiles/example-002-002-graph.dir/requires

examples/CMakeFiles/example-002-002-graph.dir/clean:
	cd /home/siqi/Documents/robia/robia/vq2-1.33/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-002-002-graph.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-002-002-graph.dir/clean

examples/CMakeFiles/example-002-002-graph.dir/depend:
	cd /home/siqi/Documents/robia/robia/vq2-1.33/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/siqi/Documents/robia/robia/vq2-1.33 /home/siqi/Documents/robia/robia/vq2-1.33/examples /home/siqi/Documents/robia/robia/vq2-1.33/build /home/siqi/Documents/robia/robia/vq2-1.33/build/examples /home/siqi/Documents/robia/robia/vq2-1.33/build/examples/CMakeFiles/example-002-002-graph.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-002-002-graph.dir/depend

