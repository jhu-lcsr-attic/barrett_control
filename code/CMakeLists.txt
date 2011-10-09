
set( 
  REQUIRED_CISST_LIBRARIES 
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )
find_package( sawCANBus REQUIRED HINTS "${CISST_BINARY_DIR}/saw/components")

if( cisst_FOUND AND sawCANBus_FOUND )

  # load cisst configuration
  include (${CISST_USE_FILE})
  
  set( 
    HEADER_FILES 
    ../Barrett.h
    ../osaPuck.h
    ../osaGroup.h
    ../osaWAM.h
    ../mtsPuck.h
    ../mtsWAM.h )
  
  set( 
    SOURCE_FILES 
    osaPuck.cpp
    osaGroup.cpp
    osaWAM.cpp
    mtsPuck.cpp
    mtsWAM.cpp )
  
  
  add_library( sawBarrett ${HEADER_FILES} ${SOURCE_FILES} )
  cisst_target_link_libraries( sawBarrett ${REQUIRED_CISST_LIBRARIES} )

  # create/configure file for find_package (sawNDITracker)
  set(sawBarrett_INCLUDE_DIR "${sawComponents_SOURCE_DIR}" "${sawComponents_BINARY_DIR}")
  set(sawBarrett_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set(sawBarrett_LIBRARIES sawBarrett)

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sawBarrettConfig.cmake.in"
    "${sawComponents_BINARY_DIR}/sawBarrettConfig.cmake"
    @ONLY@ )

endif( cisst_FOUND AND sawCANBus_FOUND )
