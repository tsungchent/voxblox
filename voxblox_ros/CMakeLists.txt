cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

find_package(ament_cmake REQUIRED)

# add_definitions(-std=c++14 -Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

find_package(cv_bridge REQUIRED)
find_package(gflags_vendor REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(minkindr_conversions REQUIRED)
find_package(pcl_vendor REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(voxblox_rviz_plugin REQUIRED)


#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  # src/esdf_server.cc
  src/interactive_slider.cc
  # src/simulation_server.cc
  # src/intensity_server.cc
  src/transformer.cc
  # src/tsdf_server.cc
)

ament_target_dependencies(
  ${PROJECT_NAME}
  cv_bridge
  gflags_vendor
  interactive_markers
  minkindr_conversions
  pcl_vendor
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf2
  voxblox
  voxblox_msgs
  voxblox_rviz_plugin
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
)

############
# BINARIES #
############

# add_executable(voxblox_eval
#   src/voxblox_eval.cc
# )
# target_link_libraries(voxblox_eval ${PROJECT_NAME})

# add_executable(tsdf_server
#   src/tsdf_server_node.cc
# )
# target_link_libraries(tsdf_server ${PROJECT_NAME})

# add_executable(esdf_server
#   src/esdf_server_node.cc
# )
# target_link_libraries(esdf_server ${PROJECT_NAME})

# add_executable(intensity_server
#   src/intensity_server_node.cc
# )
# target_link_libraries(intensity_server ${PROJECT_NAME})

# add_executable(simulation_eval
#   src/simulation_eval.cc
# )
# target_link_libraries(simulation_eval ${PROJECT_NAME})

# add_executable(visualize_tsdf
#   src/visualize_tsdf.cc
# )
# target_link_libraries(visualize_tsdf ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} 
# voxblox_eval 
# tsdf_server esdf_server intensity_server simulation_eval visualize_tsdf 
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

##########
# EXPORT #
##########
ament_package()
