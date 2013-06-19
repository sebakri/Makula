project(Makula)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

add_definitions(-std=c++11)

find_package(Doxygen)

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
set(TBB_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rdparty/TBB/include/)
set(TBB_LIBRARY_DIRS ${CMAKE_SOURCE_DIR}/3rdparty/TBB/lib/intel64/gcc4.4/)

include_directories(
  ${TBB_INCLUDE_DIRS}
)

link_directories(${TBB_LIBRARY_DIRS})

find_library(TBB_LIBRARIES
  NAMES tbb tbbmalloc
  PATHS ${TBB_LIBRARY_DIRS}
)

list(APPEND MAKULA_LIBRARIES ${TBB_LIBRARIES})

include_directories(include)

if(BUILD_TESTS)

  include(GetGMock)
  
  include_directories(${GMOCK_INCLUDE_DIRS})
  link_directories(${GMOCK_LIB_DIRS})
  
  enable_testing()
  
  
  
  add_custom_target(run_makula_tests
    COMMAND ctest -V
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_subdirectory(test)
  
endif()

add_subdirectory(src)
add_subdirectory(examples)