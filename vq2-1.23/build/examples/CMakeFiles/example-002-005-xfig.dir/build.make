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
CMAKE_SOURCE_DIR = /home/me/Documents/robia/vq2-1.23

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/me/Documents/robia/vq2-1.23/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-002-005-xfig.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example-002-005-xfig.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-002-005-xfig.dir/flags.make

examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o: examples/CMakeFiles/example-002-005-xfig.dir/flags.make
examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o: ../examples/example-002-005-xfig.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/me/Documents/robia/vq2-1.23/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o"
	cd /home/me/Documents/robia/vq2-1.23/build/examples && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o -c /home/me/Documents/robia/vq2-1.23/examples/example-002-005-xfig.cc

examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.i"
	cd /home/me/Documents/robia/vq2-1.23/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/me/Documents/robia/vq2-1.23/examples/example-002-005-xfig.cc > CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.i

examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.s"
	cd /home/me/Documents/robia/vq2-1.23/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/me/Documents/robia/vq2-1.23/examples/example-002-005-xfig.cc -o CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.s

examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.requires:
.PHONY : examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.requires

examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.provides: examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.requires
	$(MAKE) -f examples/CMakeFiles/example-002-005-xfig.dir/build.make examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.provides.build
.PHONY : examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.provides

examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.provides.build: examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o

# Object files for target example-002-005-xfig
example__002__005__xfig_OBJECTS = \
"CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o"

# External object files for target example-002-005-xfig
example__002__005__xfig_EXTERNAL_OBJECTS =

examples/example-002-005-xfig: examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o
examples/example-002-005-xfig: examples/CMakeFiles/example-002-005-xfig.dir/build.make
examples/example-002-005-xfig: examples/CMakeFiles/example-002-005-xfig.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable example-002-005-xfig"
	cd /home/me/Documents/robia/vq2-1.23/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-002-005-xfig.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-002-005-xfig.dir/build: examples/example-002-005-xfig
.PHONY : examples/CMakeFiles/example-002-005-xfig.dir/build

examples/CMakeFiles/example-002-005-xfig.dir/requires: examples/CMakeFiles/example-002-005-xfig.dir/example-002-005-xfig.cc.o.requires
.PHONY : examples/CMakeFiles/example-002-005-xfig.dir/requires

examples/CMakeFiles/example-002-005-xfig.dir/clean:
	cd /home/me/Documents/robia/vq2-1.23/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-002-005-xfig.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-002-005-xfig.dir/clean

examples/CMakeFiles/example-002-005-xfig.dir/depend:
	cd /home/me/Documents/robia/vq2-1.23/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/me/Documents/robia/vq2-1.23 /home/me/Documents/robia/vq2-1.23/examples /home/me/Documents/robia/vq2-1.23/build /home/me/Documents/robia/vq2-1.23/build/examples /home/me/Documents/robia/vq2-1.23/build/examples/CMakeFiles/example-002-005-xfig.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-002-005-xfig.dir/depend
