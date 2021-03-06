cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(image_registration)

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()

find_package(OpenCV COMPONENTS core highgui video imgproc contrib REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options filesystem system REQUIRED )
find_package(OpenNI REQUIRED)
find_package(PCL REQUIRED)
find_package(OPENGL REQUIRED)
find_package(GLM REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(GLFW REQUIRED glfw3)

if (MSVC)
    add_definitions("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DBOOST_ALL_DYN_LINK")
endif()

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${OPENNI_INCLUDES} ${OPENNI2_INCLUDES} ${OPENNI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (image_registration main.cpp CVVideoInput.cpp ONIVideoInput.cpp Tracker.cpp Viewer.cpp PointCloudWrapper.cpp)
target_link_libraries(image_registration ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENNI_LIBRARIES} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLM_LIBRARIES})

### These settings help cmake work with Eclipse
IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# By turning on verbose, Eclipse can find locations of header files
SET(CMAKE_VERBOSE_MAKEFILE ON)
