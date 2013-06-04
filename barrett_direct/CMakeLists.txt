# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(barrett_direct)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS leo_can )

message(" ----- Xenomai: ${Xenomai_FOUND}")

find_package(catkin REQUIRED COMPONENTS xenomai_ros)

message(" ----- Xenomai post: ${Xenomai_FOUND}")

if(Xenomai_FOUND)

  find_package(Eigen REQUIRED)

  include_directories(include ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})

  set( SOURCE_FILES 
    src/Puck.cpp
    src/Group.cpp
    src/WAM.cpp
    src/BH8_280.cpp )

  # Add the library target
  add_library( barrett_direct ${SOURCE_FILES} )
  target_link_libraries( barrett_direct ${leo_can_LIBRARIES} )

  # Add example targets
  add_xenomai_flags()

  add_executable( wam_test examples/wam_test.cpp )
  target_link_libraries( wam_test barrett_direct xenomai native rtdm )

  # Declare catkin package
  catkin_package(
    DEPENDS leo_can
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
    )
else()
  message("Xenomai not found, not building barrett_direct.")
endif()

