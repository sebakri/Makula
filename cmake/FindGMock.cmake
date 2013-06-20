add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/gmock)

set(GMOCK_INCLUDE_DIRS 
  ${CMAKE_SOURCE_DIR}/3rdparty/gmock/include
  ${CMAKE_SOURCE_DIR}/3rdparty/gmock/gtest/include
)

set(GMOCK_BINARY_DIR ${CMAKE_BINARY_DIR}/3rdparty/gmock)

find_library( GMOCK_LIBRARY
  NAMES gmock gmock_main
  HINTS ${GMOCK_BINARY_DIR}
)

if(GMOCK_LIBRARY)
  set(GMOCK_FOUND 1)
  
  set( GMOCK_LIBRARY_DIRS
    ${GMOCK_BINARY_DIR}
    ${GMOCK_BINARY_DIR}/gtest
  )
  
  set( GMOCK_LIBRARIES
    gmock
    gmock_main
    gtest
    gtest_main
  )
endif()