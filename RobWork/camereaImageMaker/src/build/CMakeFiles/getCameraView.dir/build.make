# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build

# Include any dependencies generated for this target.
include CMakeFiles/getCameraView.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/getCameraView.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/getCameraView.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/getCameraView.dir/flags.make

CMakeFiles/getCameraView.dir/main.cpp.o: CMakeFiles/getCameraView.dir/flags.make
CMakeFiles/getCameraView.dir/main.cpp.o: ../main.cpp
CMakeFiles/getCameraView.dir/main.cpp.o: CMakeFiles/getCameraView.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/getCameraView.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/getCameraView.dir/main.cpp.o -MF CMakeFiles/getCameraView.dir/main.cpp.o.d -o CMakeFiles/getCameraView.dir/main.cpp.o -c /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/main.cpp

CMakeFiles/getCameraView.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getCameraView.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/main.cpp > CMakeFiles/getCameraView.dir/main.cpp.i

CMakeFiles/getCameraView.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getCameraView.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/main.cpp -o CMakeFiles/getCameraView.dir/main.cpp.s

# Object files for target getCameraView
getCameraView_OBJECTS = \
"CMakeFiles/getCameraView.dir/main.cpp.o"

# External object files for target getCameraView
getCameraView_EXTERNAL_OBJECTS =

getCameraView: CMakeFiles/getCameraView.dir/main.cpp.o
getCameraView: CMakeFiles/getCameraView.dir/build.make
getCameraView: /usr/lib/x86_64-linux-gnu/libxerces-c.so
getCameraView: /usr/lib/x86_64-linux-gnu/libOpenGL.so
getCameraView: /usr/lib/x86_64-linux-gnu/libGLX.so
getCameraView: /usr/lib/x86_64-linux-gnu/libGLU.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libyaobi.a
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libpqp.a
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_csgjs.a
getCameraView: /usr/lib/x86_64-linux-gnu/libfcl.so
getCameraView: /usr/lib/x86_64-linux-gnu/libccd.so
getCameraView: /usr/lib/x86_64-linux-gnu/libm.so
getCameraView: /usr/lib/x86_64-linux-gnu/liboctomap.so
getCameraView: /usr/lib/x86_64-linux-gnu/liboctomath.so
getCameraView: /usr/lib/x86_64-linux-gnu/libassimp.so
getCameraView: /usr/lib/x86_64-linux-gnu/libdl.a
getCameraView: /home/rovi2022/Programs/RobWork/Build/RWStudio/libs/relwithdebinfo/libsdurws_robworkstudioapp.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RWStudio/libs/relwithdebinfo/libsdurws_workcelleditor.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RWStudio/libs/relwithdebinfo/libsdurws.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RWStudio/libs/relwithdebinfo/libqtpropertybrowser.a
getCameraView: /usr/lib/x86_64-linux-gnu/libxerces-c.so
getCameraView: /usr/lib/x86_64-linux-gnu/libOpenGL.so
getCameraView: /usr/lib/x86_64-linux-gnu/libGLX.so
getCameraView: /usr/lib/x86_64-linux-gnu/libGLU.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libyaobi.a
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libpqp.a
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_csgjs.a
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_pathplanners.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_pathoptimization.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_simulation.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_opengl.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_assembly.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_task.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_calibration.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_csg.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_control.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_proximitystrategies.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_plugin.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_graspplanning.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_loaders.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_pathplanning.so
getCameraView: /usr/lib/x86_64-linux-gnu/libfcl.so
getCameraView: /usr/lib/x86_64-linux-gnu/libccd.so
getCameraView: /usr/lib/x86_64-linux-gnu/libm.so
getCameraView: /usr/lib/x86_64-linux-gnu/liboctomap.so
getCameraView: /usr/lib/x86_64-linux-gnu/liboctomath.so
getCameraView: /usr/lib/x86_64-linux-gnu/libassimp.so
getCameraView: /usr/lib/x86_64-linux-gnu/libdl.a
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libglut.so
getCameraView: /usr/lib/x86_64-linux-gnu/libpython3.10.so
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_algorithms.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_invkin.so
getCameraView: /usr/lib/x86_64-linux-gnu/libOpenGL.so
getCameraView: /usr/lib/x86_64-linux-gnu/libGLX.so
getCameraView: /usr/lib/x86_64-linux-gnu/libGLU.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libpqp.a
getCameraView: /usr/lib/x86_64-linux-gnu/libxerces-c.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_graphics.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_trajectory.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_proximity.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_models.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_sensor.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_geometry.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_kinematics.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_math.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_common.so
getCameraView: /home/rovi2022/Programs/RobWork/Build/RW/libs/relwithdebinfo/libsdurw_core.so
getCameraView: /usr/lib/gcc/x86_64-linux-gnu/11/libgomp.so
getCameraView: /usr/lib/x86_64-linux-gnu/libpthread.a
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.74.0
getCameraView: /usr/lib/x86_64-linux-gnu/libQt6OpenGLWidgets.so.6.2.4
getCameraView: /usr/lib/x86_64-linux-gnu/libQt6OpenGL.so.6.2.4
getCameraView: /usr/lib/x86_64-linux-gnu/libQt6Widgets.so.6.2.4
getCameraView: /usr/lib/x86_64-linux-gnu/libQt6Gui.so.6.2.4
getCameraView: /usr/lib/x86_64-linux-gnu/libQt6Core.so.6.2.4
getCameraView: /usr/lib/x86_64-linux-gnu/libGLX.so
getCameraView: /usr/lib/x86_64-linux-gnu/libOpenGL.so
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
getCameraView: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
getCameraView: CMakeFiles/getCameraView.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable getCameraView"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getCameraView.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/getCameraView.dir/build: getCameraView
.PHONY : CMakeFiles/getCameraView.dir/build

CMakeFiles/getCameraView.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/getCameraView.dir/cmake_clean.cmake
.PHONY : CMakeFiles/getCameraView.dir/clean

CMakeFiles/getCameraView.dir/depend:
	cd /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build /home/rovi2022/projects/RoVi_Project/camereaImageMaker/src/build/CMakeFiles/getCameraView.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/getCameraView.dir/depend

