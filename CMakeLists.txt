
set( 
  CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${CMAKE_CURRENT_SOURCE_DIR}/code/cmake/Modules
  )

add_subdirectory( code )
add_subdirectory( examples )

