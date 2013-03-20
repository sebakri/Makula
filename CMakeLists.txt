project(Makula)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.6 REQUIRED)
find_package(MRPT REQUIRED)
find_package(Doxygen)

if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc
  ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${MRPT_INCLUDE_DIRS}
  io/include)

set(SRC_LIST
  main.cpp)
  
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Specify MainTest's link libraries
target_link_libraries(${PROJECT_NAME} makula_io)

add_subdirectory(io)