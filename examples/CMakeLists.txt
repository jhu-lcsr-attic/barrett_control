

set( 
  REQUIRED_CISST_LIBRARIES 
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )
find_package( sawBarrett REQUIRED HINTS "${CISST_BINARY_DIR}/saw/components")

if( cisst_FOUND AND sawBarrett_FOUND )

  # load cisst configuration
  include( ${CISST_USE_FILE} )

  add_executable( osaPuckExample    osaPuckExample.cpp )
  add_executable( osaGroupExample   osaGroupExample.cpp )
  add_executable( osaWAMExample     osaWAMExample.cpp )

  add_executable( mtsPuckExample mtsPuckExample.cpp )
  add_executable( mtsWAMExample  mtsWAMExample.cpp )

  target_link_libraries( osaPuckExample  sawBarrett sawCANBus )
  target_link_libraries( osaGroupExample sawBarrett sawCANBus )
  target_link_libraries( osaWAMExample   sawBarrett sawCANBus )
  target_link_libraries( mtsPuckExample  sawBarrett sawCANBus )
  target_link_libraries( mtsWAMExample   sawBarrett sawCANBus )

  cisst_target_link_libraries( osaPuckExample  ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( osaGroupExample ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( osaWAMExample   ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( mtsPuckExample  ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( mtsWAMExample   ${REQUIRED_CISST_LIBRARIES} )

endif( cisst_FOUND AND sawBarrett_FOUND )
