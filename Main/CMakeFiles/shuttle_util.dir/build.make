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
CMAKE_SOURCE_DIR = /home/pi/workspace/shuttle/Main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/workspace/shuttle/Main

# Include any dependencies generated for this target.
include CMakeFiles/shuttle_util.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shuttle_util.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shuttle_util.dir/flags.make

CMakeFiles/shuttle_util.dir/shuttle_util.cpp.o: CMakeFiles/shuttle_util.dir/flags.make
CMakeFiles/shuttle_util.dir/shuttle_util.cpp.o: shuttle_util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/workspace/shuttle/Main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/shuttle_util.dir/shuttle_util.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shuttle_util.dir/shuttle_util.cpp.o -c /home/pi/workspace/shuttle/Main/shuttle_util.cpp

CMakeFiles/shuttle_util.dir/shuttle_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shuttle_util.dir/shuttle_util.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/workspace/shuttle/Main/shuttle_util.cpp > CMakeFiles/shuttle_util.dir/shuttle_util.cpp.i

CMakeFiles/shuttle_util.dir/shuttle_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shuttle_util.dir/shuttle_util.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/workspace/shuttle/Main/shuttle_util.cpp -o CMakeFiles/shuttle_util.dir/shuttle_util.cpp.s

# Object files for target shuttle_util
shuttle_util_OBJECTS = \
"CMakeFiles/shuttle_util.dir/shuttle_util.cpp.o"

# External object files for target shuttle_util
shuttle_util_EXTERNAL_OBJECTS =

libshuttle_util.a: CMakeFiles/shuttle_util.dir/shuttle_util.cpp.o
libshuttle_util.a: CMakeFiles/shuttle_util.dir/build.make
libshuttle_util.a: CMakeFiles/shuttle_util.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/workspace/shuttle/Main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libshuttle_util.a"
	$(CMAKE_COMMAND) -P CMakeFiles/shuttle_util.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shuttle_util.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shuttle_util.dir/build: libshuttle_util.a

.PHONY : CMakeFiles/shuttle_util.dir/build

CMakeFiles/shuttle_util.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shuttle_util.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shuttle_util.dir/clean

CMakeFiles/shuttle_util.dir/depend:
	cd /home/pi/workspace/shuttle/Main && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/workspace/shuttle/Main /home/pi/workspace/shuttle/Main /home/pi/workspace/shuttle/Main /home/pi/workspace/shuttle/Main /home/pi/workspace/shuttle/Main/CMakeFiles/shuttle_util.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shuttle_util.dir/depend

