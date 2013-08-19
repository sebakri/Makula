set(test_core_srcs
  core_tests.cpp
  buffer_unit_test.cpp
  channel_unit_test.cpp
  worker_pool_unit_test.cpp
  processes_unit_test.cpp
)

add_definitions(-fprofile-arcs -ftest-coverage)
add_executable(base_tests ${test_core_srcs})

target_link_libraries(base_tests ${GMOCK_LIBRARIES} pthread ${TBB_LIBRARIES} ${PCL_LIBRARIES} -fprofile-arcs)

add_test(
  NAME testingBase
  COMMAND base_tests
)