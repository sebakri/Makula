include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})

set(recognition_srcs 
  classifying.cpp 
  clustering.cpp
  object.cpp
  filtering.cpp 
  point_cloud_viewer.cpp 
  main.cpp
  localization.cpp
  point_cloud_source.cpp
)

set(benchmark_srcs 
  classifying.cpp 
  clustering.cpp
  object.cpp
  filtering.cpp 
  point_cloud_viewer.cpp 
  benchmark.cpp
  localization.cpp
  point_cloud_source_mock.cpp
)

add_executable(makula_recognition_pcl ${recognition_srcs} ${recognition_uis_h})

target_link_libraries(makula_recognition_pcl ${PCL_LIBRARIES} ${MAKULA_LIBRARIES})

add_executable(makula_recognition_pcl_benchmark ${benchmark_srcs} ${recognition_uis_h})

target_link_libraries(makula_recognition_pcl_benchmark ${PCL_LIBRARIES} ${MAKULA_LIBRARIES})