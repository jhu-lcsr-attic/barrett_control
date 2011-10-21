

set( 
  REQUIRED_CISST_LIBRARIES 
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical )

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
find_package(sawBarrett REQUIRED HINTS "${CISST_BINARY_DIR}/saw/components")

if( cisst_FOUND AND sawBarrett_FOUND )

  # load cisst configuration
  include( ${CISST_USE_FILE} )

  set( REQUIRED_SAW_COMPONENTS sawBarrett sawCANBus )

  add_executable( osaPuckExample    osaPuckExample.cpp )
  add_executable( osaGroupExample   osaGroupExample.cpp )
  add_executable( osaWAMExample     osaWAMExample.cpp )
  add_executable( osaBH8_280Example osaBH8_280Example.cpp )

  add_executable( mtsPuckExample mtsPuckExample.cpp )
  add_executable( mtsWAMExample  mtsWAMExample.cpp )

  target_link_libraries( osaPuckExample    ${REQUIRED_SAW_COMPONENTS} )
  target_link_libraries( osaGroupExample   ${REQUIRED_SAW_COMPONENTS} )
  target_link_libraries( osaWAMExample     ${REQUIRED_SAW_COMPONENTS} )
  target_link_libraries( osaBH8_280Example ${REQUIRED_SAW_COMPONENTS} )
  target_link_libraries( mtsPuckExample    ${REQUIRED_SAW_COMPONENTS} )
  target_link_libraries( mtsWAMExample     ${REQUIRED_SAW_COMPONENTS} )

  cisst_target_link_libraries( osaPuckExample    ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( osaGroupExample   ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( osaWAMExample     ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( osaBH8_280Example ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( mtsPuckExample    ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( mtsWAMExample     ${REQUIRED_CISST_LIBRARIES} )

  find_package( sawControllers HINTS "${CISST_BINARY_DIR}/saw/components" )
  find_package( sawKeyboard    HINTS "${CISST_BINARY_DIR}/saw/components" )
  if( sawControllers_FOUND AND sawKeyboard_FOUND )

    set( 
      REQUIRED_SAW_COMPONENTS 
      ${REQUIRED_SAW_COMPONENTS} 
      sawControllers
      sawKeyboard )

    add_executable( osaWAMGCExample   osaWAMGCExample.cpp )
    add_executable( osaWAMPDGCExample osaWAMPDGCExample.cpp )
    add_executable( mtsWAMGCExample   mtsWAMGCExample.cpp )
    add_executable( mtsWAMPDGCExample mtsWAMPDGCExample.cpp )

    target_link_libraries( osaWAMGCExample   ${REQUIRED_SAW_COMPONENTS} )
    target_link_libraries( osaWAMPDGCExample ${REQUIRED_SAW_COMPONENTS} )
    target_link_libraries( mtsWAMGCExample   ${REQUIRED_SAW_COMPONENTS} )
    target_link_libraries( mtsWAMPDGCExample ${REQUIRED_SAW_COMPONENTS} )

    cisst_target_link_libraries( osaWAMGCExample   ${REQUIRED_CISST_LIBRARIES} )
    cisst_target_link_libraries( osaWAMPDGCExample ${REQUIRED_CISST_LIBRARIES} )
    cisst_target_link_libraries( mtsWAMGCExample   ${REQUIRED_CISST_LIBRARIES} )
    cisst_target_link_libraries( mtsWAMPDGCExample ${REQUIRED_CISST_LIBRARIES} )

  endif( sawControllers_FOUND AND sawKeyboard_FOUND )


endif( cisst_FOUND AND sawBarrett_FOUND )
