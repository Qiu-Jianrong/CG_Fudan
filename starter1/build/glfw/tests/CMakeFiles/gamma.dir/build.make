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
CMAKE_SOURCE_DIR = /home/qiu/Desktop/starter1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qiu/Desktop/starter1/build

# Include any dependencies generated for this target.
include glfw/tests/CMakeFiles/gamma.dir/depend.make

# Include the progress variables for this target.
include glfw/tests/CMakeFiles/gamma.dir/progress.make

# Include the compile flags for this target's objects.
include glfw/tests/CMakeFiles/gamma.dir/flags.make

glfw/tests/CMakeFiles/gamma.dir/gamma.c.o: glfw/tests/CMakeFiles/gamma.dir/flags.make
glfw/tests/CMakeFiles/gamma.dir/gamma.c.o: ../glfw/tests/gamma.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiu/Desktop/starter1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object glfw/tests/CMakeFiles/gamma.dir/gamma.c.o"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gamma.dir/gamma.c.o   -c /home/qiu/Desktop/starter1/glfw/tests/gamma.c

glfw/tests/CMakeFiles/gamma.dir/gamma.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gamma.dir/gamma.c.i"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/qiu/Desktop/starter1/glfw/tests/gamma.c > CMakeFiles/gamma.dir/gamma.c.i

glfw/tests/CMakeFiles/gamma.dir/gamma.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gamma.dir/gamma.c.s"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/qiu/Desktop/starter1/glfw/tests/gamma.c -o CMakeFiles/gamma.dir/gamma.c.s

glfw/tests/CMakeFiles/gamma.dir/__/deps/getopt.c.o: glfw/tests/CMakeFiles/gamma.dir/flags.make
glfw/tests/CMakeFiles/gamma.dir/__/deps/getopt.c.o: ../glfw/deps/getopt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiu/Desktop/starter1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object glfw/tests/CMakeFiles/gamma.dir/__/deps/getopt.c.o"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gamma.dir/__/deps/getopt.c.o   -c /home/qiu/Desktop/starter1/glfw/deps/getopt.c

glfw/tests/CMakeFiles/gamma.dir/__/deps/getopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gamma.dir/__/deps/getopt.c.i"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/qiu/Desktop/starter1/glfw/deps/getopt.c > CMakeFiles/gamma.dir/__/deps/getopt.c.i

glfw/tests/CMakeFiles/gamma.dir/__/deps/getopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gamma.dir/__/deps/getopt.c.s"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/qiu/Desktop/starter1/glfw/deps/getopt.c -o CMakeFiles/gamma.dir/__/deps/getopt.c.s

glfw/tests/CMakeFiles/gamma.dir/__/deps/glad.c.o: glfw/tests/CMakeFiles/gamma.dir/flags.make
glfw/tests/CMakeFiles/gamma.dir/__/deps/glad.c.o: ../glfw/deps/glad.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiu/Desktop/starter1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object glfw/tests/CMakeFiles/gamma.dir/__/deps/glad.c.o"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gamma.dir/__/deps/glad.c.o   -c /home/qiu/Desktop/starter1/glfw/deps/glad.c

glfw/tests/CMakeFiles/gamma.dir/__/deps/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gamma.dir/__/deps/glad.c.i"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/qiu/Desktop/starter1/glfw/deps/glad.c > CMakeFiles/gamma.dir/__/deps/glad.c.i

glfw/tests/CMakeFiles/gamma.dir/__/deps/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gamma.dir/__/deps/glad.c.s"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/qiu/Desktop/starter1/glfw/deps/glad.c -o CMakeFiles/gamma.dir/__/deps/glad.c.s

# Object files for target gamma
gamma_OBJECTS = \
"CMakeFiles/gamma.dir/gamma.c.o" \
"CMakeFiles/gamma.dir/__/deps/getopt.c.o" \
"CMakeFiles/gamma.dir/__/deps/glad.c.o"

# External object files for target gamma
gamma_EXTERNAL_OBJECTS =

glfw/tests/gamma: glfw/tests/CMakeFiles/gamma.dir/gamma.c.o
glfw/tests/gamma: glfw/tests/CMakeFiles/gamma.dir/__/deps/getopt.c.o
glfw/tests/gamma: glfw/tests/CMakeFiles/gamma.dir/__/deps/glad.c.o
glfw/tests/gamma: glfw/tests/CMakeFiles/gamma.dir/build.make
glfw/tests/gamma: glfw/src/libglfw3.a
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/librt.so
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/libm.so
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/libX11.so
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/libXrandr.so
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/libXinerama.so
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
glfw/tests/gamma: /usr/lib/x86_64-linux-gnu/libXcursor.so
glfw/tests/gamma: glfw/tests/CMakeFiles/gamma.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiu/Desktop/starter1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C executable gamma"
	cd /home/qiu/Desktop/starter1/build/glfw/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gamma.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
glfw/tests/CMakeFiles/gamma.dir/build: glfw/tests/gamma

.PHONY : glfw/tests/CMakeFiles/gamma.dir/build

glfw/tests/CMakeFiles/gamma.dir/clean:
	cd /home/qiu/Desktop/starter1/build/glfw/tests && $(CMAKE_COMMAND) -P CMakeFiles/gamma.dir/cmake_clean.cmake
.PHONY : glfw/tests/CMakeFiles/gamma.dir/clean

glfw/tests/CMakeFiles/gamma.dir/depend:
	cd /home/qiu/Desktop/starter1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiu/Desktop/starter1 /home/qiu/Desktop/starter1/glfw/tests /home/qiu/Desktop/starter1/build /home/qiu/Desktop/starter1/build/glfw/tests /home/qiu/Desktop/starter1/build/glfw/tests/CMakeFiles/gamma.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : glfw/tests/CMakeFiles/gamma.dir/depend

