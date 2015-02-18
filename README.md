Point Cloud Projection
======================

Given a point cloud (depth map) and image from one camera, project the point cloud
into a devices reference frame, using the device's camera and sensors.


Dependencies
------------

Required: OpenCV, PCL, OpenNI, Boost
Visualization: GLFW, GLM, OpenGL
Build: CMake

To build the project as is, you must have all of these dependencies installed. You can
replace Viewer.cpp and Viewer.h to reduce some of the dependencies.

Building
--------
mkdir build
cd build
cmake ..


Setup project with Eclipse
--------------------------

Based on: http://www.vtk.org/Wiki/CMake:Eclipse_UNIX_Tutorial
- Clone repo
- cd into the repo directory and run the following commands:
    - mkdir build
    - cd build
    - cmake ..
- Create a new Eclipse workspace
- Switch to the C/C++ perspective
- File -> Import -> C/C++ -> Existing code as Makefile Project
    - Select project directory and give project a name
    - Choose <none> for Toolchain for indexer
- Right click the project and go to Properties -> C/C++ build
    - Set the build command to "make -C ${ProjDirPath}/build VERBOSE=1"
    - Under C/C++ Build go to "Settings" and select Mach-O parser in the Binary Parsers dialog
    - Under C/C++ Build go to "Preprocessor Include Path"
        - Under the Providers tab select:
            - CDT User Setting Entries
            - Exported Entries from Referenced Projects
            - CDT Managed Build Settings Entries
            - CDT GCC Build Output Parser
            - CDT GCC Built-in Compiler Settings
- Now rebuild the project and check the Console for any errors
- You may have to run Project -> C/C++ Index -> Rebuild to get rid of Eclipse errors
