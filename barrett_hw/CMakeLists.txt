# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(barrett_hw)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin COMPONENTS hardware_interface libbarrett 
  barrett_model kdl_urdf_tools terse_roscpp controller_manager
  barrett_controllers control_toolbox rospy xenomai_ros)

if(Xenomai_FOUND)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  # CATKIN_MIGRATION: removed during catkin migration
  # include(${rosbuild_xenomai_PACKAGE_PATH}/rosbuild_xenomai.cmake)

  find_package(Eigen REQUIRED)
  include_directories(${EIGEN_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})

  # Add xenomai flags
  add_xenomai_flags()

  add_library(barrett_hw src/wam.cpp include/barrett_hw/wam.h)

  #add_definitions(-D__XENO__)
  add_executable(wam_server src/wam_server.cpp)
  target_link_libraries(wam_server xenomai native rtdm barrett_hw)
  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
  )
  # TODO: fill in what other packages will need to use this package
  ## DEPENDS: system dependencies of this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## INCLUDE_DIRS: 
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  catkin_package(
      DEPENDS hardware_interface libbarrett barrett_model kdl_urdf_tools terse_roscpp controller_manager barrett_controllers control_toolbox rospy
      CATKIN_DEPENDS # TODO
      INCLUDE_DIRS # TODO include
      LIBRARIES # TODO
  )

endif()
