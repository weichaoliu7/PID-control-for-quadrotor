# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = D:/CMake/bin/cmake.exe

# The command to remove a file.
RM = D:/CMake/bin/cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/build

# Include any dependencies generated for this target.
include CMakeFiles/pid.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pid.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pid.dir/flags.make

CMakeFiles/pid.dir/src/pid.c.obj: CMakeFiles/pid.dir/flags.make
CMakeFiles/pid.dir/src/pid.c.obj: CMakeFiles/pid.dir/includes_C.rsp
CMakeFiles/pid.dir/src/pid.c.obj: D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/src/pid.c
CMakeFiles/pid.dir/src/pid.c.obj: CMakeFiles/pid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/pid.dir/src/pid.c.obj"
	E:/Webots/msys64/mingw64/bin/cc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pid.dir/src/pid.c.obj -MF CMakeFiles/pid.dir/src/pid.c.obj.d -o CMakeFiles/pid.dir/src/pid.c.obj -c D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/src/pid.c

CMakeFiles/pid.dir/src/pid.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/pid.dir/src/pid.c.i"
	E:/Webots/msys64/mingw64/bin/cc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/src/pid.c > CMakeFiles/pid.dir/src/pid.c.i

CMakeFiles/pid.dir/src/pid.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/pid.dir/src/pid.c.s"
	E:/Webots/msys64/mingw64/bin/cc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/src/pid.c -o CMakeFiles/pid.dir/src/pid.c.s

# Object files for target pid
pid_OBJECTS = \
"CMakeFiles/pid.dir/src/pid.c.obj"

# External object files for target pid
pid_EXTERNAL_OBJECTS =

D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe: CMakeFiles/pid.dir/src/pid.c.obj
D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe: CMakeFiles/pid.dir/build.make
D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe: CMakeFiles/pid.dir/linkLibs.rsp
D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe: CMakeFiles/pid.dir/objects1.rsp
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe"
	D:/CMake/bin/cmake.exe -E rm -f CMakeFiles/pid.dir/objects.a
	E:/Webots/msys64/mingw64/bin/ar.exe qc CMakeFiles/pid.dir/objects.a @CMakeFiles/pid.dir/objects1.rsp
	E:/Webots/msys64/mingw64/bin/cc.exe -g -Wl,--whole-archive CMakeFiles/pid.dir/objects.a -Wl,--no-whole-archive -o D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe -Wl,--out-implib,D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/libpid.dll.a -Wl,--major-image-version,0,--minor-image-version,0 @CMakeFiles/pid.dir/linkLibs.rsp

# Rule to build all files generated by this target.
CMakeFiles/pid.dir/build: D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/pid.exe
.PHONY : CMakeFiles/pid.dir/build

CMakeFiles/pid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pid.dir/clean

CMakeFiles/pid.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/build D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/build D:/graduate/vsCode/C/PID-control-for-quadrotor/webots/controllers/pid/build/CMakeFiles/pid.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/pid.dir/depend

