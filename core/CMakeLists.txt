find_package(TBB REQUIRED)

include_directories(
  include/
  ${TBB_INCLUDE_DIRS}
)

link_directories(${TBB_LIBRARY_DIRS})

if(BUILD_TESTS)
  add_subdirectory(test)
endif()