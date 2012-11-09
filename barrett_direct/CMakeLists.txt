cmake_minimum_required(VERSION 2.6.0)
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
rosbuild_find_ros_package(rosbuild_xenomai)
include(${rosbuild_xenomai_PACKAGE_PATH}/rosbuild_xenomai.cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

 
# Set the executable and library output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set( SOURCE_FILES 
  src/Puck.cpp
  src/Group.cpp
  src/WAM.cpp
  src/BH8_280.cpp )

# Add the library target
rosbuild_add_library( barrett_direct ${SOURCE_FILES} )
target_link_libraries( barrett_direct ${leoCAN_LIBRARIES} )

# Add example targets
rosbuild_add_xenomai_flags()

add_executable( wam_test examples/wam_test.cpp )
target_link_libraries( wam_test barrett_direct xenomai native rtdm )
