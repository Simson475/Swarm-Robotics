# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/widok/Desktop/newestDat7/P7-kode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/widok/Desktop/newestDat7/P7-kode/build

# Include any dependencies generated for this target.
include lib/jsoncpp/example/CMakeFiles/readFromString.dir/depend.make

# Include the progress variables for this target.
include lib/jsoncpp/example/CMakeFiles/readFromString.dir/progress.make

# Include the compile flags for this target's objects.
include lib/jsoncpp/example/CMakeFiles/readFromString.dir/flags.make

lib/jsoncpp/example/CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.o: lib/jsoncpp/example/CMakeFiles/readFromString.dir/flags.make
lib/jsoncpp/example/CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.o: ../lib/jsoncpp/example/readFromString/readFromString.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/widok/Desktop/newestDat7/P7-kode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/jsoncpp/example/CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.o"
	cd /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example && /usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.o -c /home/widok/Desktop/newestDat7/P7-kode/lib/jsoncpp/example/readFromString/readFromString.cpp

lib/jsoncpp/example/CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.i"
	cd /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/widok/Desktop/newestDat7/P7-kode/lib/jsoncpp/example/readFromString/readFromString.cpp > CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.i

lib/jsoncpp/example/CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.s"
	cd /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/widok/Desktop/newestDat7/P7-kode/lib/jsoncpp/example/readFromString/readFromString.cpp -o CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.s

# Object files for target readFromString
readFromString_OBJECTS = \
"CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.o"

# External object files for target readFromString
readFromString_EXTERNAL_OBJECTS =

bin/readFromString: lib/jsoncpp/example/CMakeFiles/readFromString.dir/readFromString/readFromString.cpp.o
bin/readFromString: lib/jsoncpp/example/CMakeFiles/readFromString.dir/build.make
bin/readFromString: lib/libjsoncpp.a
bin/readFromString: lib/jsoncpp/example/CMakeFiles/readFromString.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/widok/Desktop/newestDat7/P7-kode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/readFromString"
	cd /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/readFromString.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/jsoncpp/example/CMakeFiles/readFromString.dir/build: bin/readFromString

.PHONY : lib/jsoncpp/example/CMakeFiles/readFromString.dir/build

lib/jsoncpp/example/CMakeFiles/readFromString.dir/clean:
	cd /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example && $(CMAKE_COMMAND) -P CMakeFiles/readFromString.dir/cmake_clean.cmake
.PHONY : lib/jsoncpp/example/CMakeFiles/readFromString.dir/clean

lib/jsoncpp/example/CMakeFiles/readFromString.dir/depend:
	cd /home/widok/Desktop/newestDat7/P7-kode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/widok/Desktop/newestDat7/P7-kode /home/widok/Desktop/newestDat7/P7-kode/lib/jsoncpp/example /home/widok/Desktop/newestDat7/P7-kode/build /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example /home/widok/Desktop/newestDat7/P7-kode/build/lib/jsoncpp/example/CMakeFiles/readFromString.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/jsoncpp/example/CMakeFiles/readFromString.dir/depend

