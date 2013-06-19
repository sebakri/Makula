add_executable(makula_example_buffer example_buffer.cpp)
target_link_libraries(makula_example_buffer ${MAKULA_LIBRARIES})

add_executable(makula_example_channel example_channel.cpp)
target_link_libraries(makula_example_channel ${MAKULA_LIBRARIES})


add_executable(makula_example_process example_process.cpp)
target_link_libraries(makula_example_process ${MAKULA_LIBRARIES})