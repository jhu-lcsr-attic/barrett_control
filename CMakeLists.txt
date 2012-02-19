if(0)
  cmake_minimum_required(VERSION 2.4.6)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  #set(ROS_BUILD_TYPE RelWithDebInfo)

  rosbuild_init()

  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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
endif()
 
project(barrett_direct)

set( INCLUDE_DIR "include/barrett_direct" )
set( SRC_DIR "src" )

message("barrett_direct_SOURCE_DIR ${barrett_direct_SOURCE_DIR}")

set( barrett_direct_INCLUDE_DIRS "${barrett_direct_SOURCE_DIR}/include" )
set( barrett_direct_LIBRARIES_PATH )
set( barrett_direct_LIBRARIES )
set( barrett_direct_DEFINITIONS )

set( HEADER_FILES 
  ${INCLUDE_DIR}/CANBus.h 
  ${INCLUDE_DIR}/CANBusFrame.h )

set( SOURCE_FILES 
  ${SRC_DIR}/CANBus.cpp
  ${SRC_DIR}/CANBusFrame.cpp )

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

find_package(Eigen REQUIRED)

add_library( barrett_direct ${HEADER_FILES} ${SOURCE_FILES} )
include_directories(
  "${barrett_direct_SOURCE_DIR}/include"
  ${Eigen_INCLUDE_DIRS}
  "${barrett_direct_SOURCE_DIR}/../leoCAN/include")
link_directories("${barrett_direct_SOURCE_DIR}/../leoCAN/build")
target_link_libraries( barrett_direct leoCAN )
