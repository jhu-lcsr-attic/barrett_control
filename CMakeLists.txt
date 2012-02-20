cmake_minimum_required(VERSION 2.6.0)

set(ROS_ROOT $ENV{ROS_ROOT})
if(ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  #set(ROS_BUILD_TYPE RelWithDebInfo)

  rosbuild_init()

  #uncomment if you have defined messages
  #rosbuild_genmsg()
  #uncomment if you have defined services
  #rosbuild_gensrv()

  #common commands for building c++ executables and libraries
  #rosbuild_add_library(barrett_direct src/osaGroup.cpp src/osaPuck.cpp src/osaWAM.cpp src/osaBH8_280.cpp)
  #target_link_libraries(${PROJECT_NAME} another_library)
  #rosbuild_add_boost_directories()
  #rosbuild_link_boost(${PROJECT_NAME} thread)
  #rosbuild_add_executable(example examples/example.cpp)
  #target_link_libraries(example ${PROJECT_NAME})

  # Find the leoCAN ROS package
  rosbuild_find_ros_package(leoCAN)
endif()
 
project(barrett_direct)

set( INCLUDE_DIR "include/barrett_direct" )
set( SRC_DIR "src" )
set( EXAMPLES_DIR "examples" )

# Set the executable and library output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set( barrett_direct_INCLUDE_DIRS "${barrett_direct_SOURCE_DIR}/include" )
set( barrett_direct_LIBRARIES_PATH )
set( barrett_direct_LIBRARIES )
set( barrett_direct_DEFINITIONS )

# Add path to cmake modules in this package
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${barrett_direct_SOURCE_DIR}/cmake")

set( HEADER_FILES 
  ${INCLUDE_DIR}/Barrett.h
  ${INCLUDE_DIR}/Puck.h
  ${INCLUDE_DIR}/Group.h
  ${INCLUDE_DIR}/WAM.h
  ${INCLUDE_DIR}/BH8_280.h )

set( SOURCE_FILES 
  ${SRC_DIR}/Puck.cpp
  ${SRC_DIR}/Group.cpp
  ${SRC_DIR}/WAM.cpp
  ${SRC_DIR}/BH8_280.cpp )

# Find the Eigen linear algebra package
find_package(Eigen REQUIRED)

# Find the leoCAN library
find_package(leoCAN REQUIRED HINTS "${leoCAN_PACKAGE_PATH}/share/cmake")
if(leoCAN_FOUND)
  list(APPEND barrett_direct_INCLUDE_DIRS ${leoCAN_INCLUDE_DIRS} )
  list(APPEND barrett_direct_LIBRARY_DIRS ${leoCAN_LIBRARY_DIRS} )
  list(APPEND barrett_direct_LIBRARIES ${leoCAN_LIBRARIES} )
  list(APPEND barrett_direct_DEFINITIONS ${leoCAN_DEFINITIONS} )
endif()

# Add the library target
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${barrett_direct_INCLUDE_DIRS}
  "${barrett_direct_SOURCE_DIR}/include"
  "${barrett_direct_SOURCE_DIR}/../leoCAN/include")
link_directories( ${barrett_direct_LIBRARY_DIRS} )
add_definitions( ${barrett_direct_DEFINITIONS} )

add_library( barrett_direct SHARED ${HEADER_FILES} ${SOURCE_FILES} )
target_link_libraries( barrett_direct ${leoCAN_LIBRARIES} )

# Add example targets
add_executable( wam_test ${EXAMPLES_DIR}/wam_example.cpp )
target_link_libraries( wam_test barrett_direct ${leoCAN_LIBRARIES} native )
