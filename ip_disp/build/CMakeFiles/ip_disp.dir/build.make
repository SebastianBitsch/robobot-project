# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/local/svn/robobot/ip_disp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/local/svn/robobot/ip_disp/build

# Include any dependencies generated for this target.
include CMakeFiles/ip_disp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ip_disp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ip_disp.dir/flags.make

CMakeFiles/ip_disp.dir/src/main.cpp.o: CMakeFiles/ip_disp.dir/flags.make
CMakeFiles/ip_disp.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/svn/robobot/ip_disp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ip_disp.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ip_disp.dir/src/main.cpp.o -c /home/local/svn/robobot/ip_disp/src/main.cpp

CMakeFiles/ip_disp.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ip_disp.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/svn/robobot/ip_disp/src/main.cpp > CMakeFiles/ip_disp.dir/src/main.cpp.i

CMakeFiles/ip_disp.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ip_disp.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/svn/robobot/ip_disp/src/main.cpp -o CMakeFiles/ip_disp.dir/src/main.cpp.s

CMakeFiles/ip_disp.dir/src/steensy.cpp.o: CMakeFiles/ip_disp.dir/flags.make
CMakeFiles/ip_disp.dir/src/steensy.cpp.o: ../src/steensy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/svn/robobot/ip_disp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ip_disp.dir/src/steensy.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ip_disp.dir/src/steensy.cpp.o -c /home/local/svn/robobot/ip_disp/src/steensy.cpp

CMakeFiles/ip_disp.dir/src/steensy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ip_disp.dir/src/steensy.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/svn/robobot/ip_disp/src/steensy.cpp > CMakeFiles/ip_disp.dir/src/steensy.cpp.i

CMakeFiles/ip_disp.dir/src/steensy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ip_disp.dir/src/steensy.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/svn/robobot/ip_disp/src/steensy.cpp -o CMakeFiles/ip_disp.dir/src/steensy.cpp.s

CMakeFiles/ip_disp.dir/src/uservice.cpp.o: CMakeFiles/ip_disp.dir/flags.make
CMakeFiles/ip_disp.dir/src/uservice.cpp.o: ../src/uservice.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/svn/robobot/ip_disp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ip_disp.dir/src/uservice.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ip_disp.dir/src/uservice.cpp.o -c /home/local/svn/robobot/ip_disp/src/uservice.cpp

CMakeFiles/ip_disp.dir/src/uservice.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ip_disp.dir/src/uservice.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/svn/robobot/ip_disp/src/uservice.cpp > CMakeFiles/ip_disp.dir/src/uservice.cpp.i

CMakeFiles/ip_disp.dir/src/uservice.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ip_disp.dir/src/uservice.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/svn/robobot/ip_disp/src/uservice.cpp -o CMakeFiles/ip_disp.dir/src/uservice.cpp.s

CMakeFiles/ip_disp.dir/src/utime.cpp.o: CMakeFiles/ip_disp.dir/flags.make
CMakeFiles/ip_disp.dir/src/utime.cpp.o: ../src/utime.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/svn/robobot/ip_disp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ip_disp.dir/src/utime.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ip_disp.dir/src/utime.cpp.o -c /home/local/svn/robobot/ip_disp/src/utime.cpp

CMakeFiles/ip_disp.dir/src/utime.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ip_disp.dir/src/utime.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/svn/robobot/ip_disp/src/utime.cpp > CMakeFiles/ip_disp.dir/src/utime.cpp.i

CMakeFiles/ip_disp.dir/src/utime.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ip_disp.dir/src/utime.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/svn/robobot/ip_disp/src/utime.cpp -o CMakeFiles/ip_disp.dir/src/utime.cpp.s

CMakeFiles/ip_disp.dir/src/sgpiod.cpp.o: CMakeFiles/ip_disp.dir/flags.make
CMakeFiles/ip_disp.dir/src/sgpiod.cpp.o: ../src/sgpiod.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/svn/robobot/ip_disp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ip_disp.dir/src/sgpiod.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ip_disp.dir/src/sgpiod.cpp.o -c /home/local/svn/robobot/ip_disp/src/sgpiod.cpp

CMakeFiles/ip_disp.dir/src/sgpiod.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ip_disp.dir/src/sgpiod.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/svn/robobot/ip_disp/src/sgpiod.cpp > CMakeFiles/ip_disp.dir/src/sgpiod.cpp.i

CMakeFiles/ip_disp.dir/src/sgpiod.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ip_disp.dir/src/sgpiod.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/svn/robobot/ip_disp/src/sgpiod.cpp -o CMakeFiles/ip_disp.dir/src/sgpiod.cpp.s

# Object files for target ip_disp
ip_disp_OBJECTS = \
"CMakeFiles/ip_disp.dir/src/main.cpp.o" \
"CMakeFiles/ip_disp.dir/src/steensy.cpp.o" \
"CMakeFiles/ip_disp.dir/src/uservice.cpp.o" \
"CMakeFiles/ip_disp.dir/src/utime.cpp.o" \
"CMakeFiles/ip_disp.dir/src/sgpiod.cpp.o"

# External object files for target ip_disp
ip_disp_EXTERNAL_OBJECTS =

ip_disp: CMakeFiles/ip_disp.dir/src/main.cpp.o
ip_disp: CMakeFiles/ip_disp.dir/src/steensy.cpp.o
ip_disp: CMakeFiles/ip_disp.dir/src/uservice.cpp.o
ip_disp: CMakeFiles/ip_disp.dir/src/utime.cpp.o
ip_disp: CMakeFiles/ip_disp.dir/src/sgpiod.cpp.o
ip_disp: CMakeFiles/ip_disp.dir/build.make
ip_disp: CMakeFiles/ip_disp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/local/svn/robobot/ip_disp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable ip_disp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ip_disp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ip_disp.dir/build: ip_disp

.PHONY : CMakeFiles/ip_disp.dir/build

CMakeFiles/ip_disp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ip_disp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ip_disp.dir/clean

CMakeFiles/ip_disp.dir/depend:
	cd /home/local/svn/robobot/ip_disp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/local/svn/robobot/ip_disp /home/local/svn/robobot/ip_disp /home/local/svn/robobot/ip_disp/build /home/local/svn/robobot/ip_disp/build /home/local/svn/robobot/ip_disp/build/CMakeFiles/ip_disp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ip_disp.dir/depend

