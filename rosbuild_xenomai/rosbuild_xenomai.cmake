
# Add cmake module path
list(APPEND CMAKE_MODULE_PATH ${rosbuild_xenomai_PACKAGE_PATH})

# Find the Xenomai package
find_package(Xenomai REQUIRED)

macro(rosbuild_add_xenomai_flags)
  # Add definitions
  add_definitions(${Xenomai_DEFINITIONS})

  # Add link directories 
  link_directories(${Xenomai_LIBRARY_DIRS})

  # Add include directories
  include_directories(${Xenomai_INCLUDE_DIR})

  # Add libraries
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${Xenomai_LDFLAGS_NATIVE}")
endmacro(rosbuild_add_xenomai_flags)

macro(rosbuild_add_xenomai_posix_flags)
  # Add definitions
  add_definitions(${Xenomai_DEFINITIONS_POSIX})

  # Add link directories 
  link_directories(${Xenomai_LIBRARY_DIRS})

  # Add include directories
  include_directories(${Xenomai_INCLUDE_POSIX_DIR})

  # Add linker flags
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${Xenomai_LDFLAGS_POSIX}")
endmacro(rosbuild_add_xenomai_posix_flags)

#macro(rosbuild_link_xenomai ... )
#  if(Xenomai_FOUND)
#    foreach(lib_name IN ${ARGV})
#      if(lib_name STREQUAL "native")
#        target_link_libraries(${ARG1} ${Xenomai_LIBRARY_NATIVE})
#      elseif(lib_name STREQUAL "xenomai")
#        target_link_libraries(${ARG1} ${Xenomai_XENOMAI})
#      elseif(lib_name STREQUAL "rtdm")
#        target_link_libraries(${ARG1} ${Xenomai_RTDM})
#      elseif(lib_name STREQUAL "pthread_rt")
#        target_link_libraries(${ARG1} ${Xenomai_PTHREAD_RT})
#      endif()
#    endforeach(lib_name)
#  endif()
#endmacro(rosbuild_link_xenomai)
