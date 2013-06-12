set(test_core_srcs
  core_tests.cpp
  buffer_unit_test.cpp
  channel_unit_test.cpp
#   worker_pool_unit_test.cpp
#   processes_unit_test.cpp
)

add_executable(core_tests ${test_core_srcs})

target_link_libraries(core_tests ${GMOCK_LIBRARIES} pthread ${TBB_LIBRARIES})

add_test(
  NAME testingCore
  COMMAND core_tests
)