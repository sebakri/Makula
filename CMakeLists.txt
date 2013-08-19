project(Makula)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

add_definitions(-std=c++11)

find_package(Doxygen)
find_package(PCL 1.7 REQUIRED)

if(DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/Doxyfile.in ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(
    doc
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating API documentation" VERBATIM
  )
endif()


set(MAKULA_LIBRARIES)
set(TBB_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rdparty/tbb/include/)
set(TBB_LIBRARY_DIRS ${CMAKE_SOURCE_DIR}/3rdparty/tbb/lib/intel64/gcc4.4/)

include_directories(
  ${TBB_INCLUDE_DIRS}
)

link_directories(${TBB_LIBRARY_DIRS})

find_library(TBB_LIBRARIES
  NAMES tbb
  PATHS ${TBB_LIBRARY_DIRS}
)

list(APPEND MAKULA_LIBRARIES ${TBB_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

if(BUILD_TESTS)

  find_package(GMock)
  
  if(GMOCK_FOUND)
    include_directories(${GMOCK_INCLUDE_DIRS})
    link_directories(${GMOCK_LIBRARY_DIRS})
  
    enable_testing()
  
    add_custom_target(run_makula_tests
      COMMAND ctest -V
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    add_subdirectory(test)
  endif()
endif()

add_subdirectory(src)
add_subdirectory(examples)

install(
  FILES "${CMAKE_SOURCE_DIR}/include/makula/base/*.h"
  DESTINATION /include/makula/base/
)

install(
  FILES "${CMAKE_SOURCE_DIR}/include/makula/recognition/*.h"
  DESTINATION /include/makula/recognition
)

install(
  FILES "${CMAKE_SOURCE_DIR}/include/makula/recognition/pcl/*.h"
  DESTINATION /include/makula/recognition/pcl
)