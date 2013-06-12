project(Makula)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

add_definitions(-std=c++11)

find_package(TBB REQUIRED)

if(TBB_FOUND)
  include_directories(
    ${TBB_INCLUDE_DIRS}
  )

  link_directories(${TBB_LIBRARY_DIRS})

endif()

if(BUILD_TESTS)

  include(GetGMock)
  
  include_directories(${GMOCK_INCLUDE_DIRS})
  link_directories(${GMOCK_LIB_DIRS})
  
  enable_testing()
  
endif()

include_directories(include)

# add_subdirectory(core)
add_subdirectory(test)