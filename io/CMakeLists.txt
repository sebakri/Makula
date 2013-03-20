configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

set(IO_SRC_LIST 
  src/pclkinectstream.cpp)

add_library(makula_io ${IO_SRC_LIST})

target_link_libraries(makula_io ${PCL_LIBRARIES})