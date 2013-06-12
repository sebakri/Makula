project(Makula)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

add_definitions(-std=c++11)

if(BUILD_TESTS)

  include(GetGMock)
  
  include_directories(${GMOCK_INCLUDE_DIRS})
  link_directories(${GMOCK_LIB_DIRS})
  
  enable_testing()
  
endif()

add_subdirectory(core)