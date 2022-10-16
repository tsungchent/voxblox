cmake_minimum_required(VERSION 3.21)
project(voxblox_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
    "msg/Block.msg"
    "msg/Layer.msg"
    "msg/Mesh.msg"
    "msg/MeshBlock.msg"
    "msg/VoxelEvaluationDetails.msg"
)
set(srv_files
 "srv/FilePath.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

##########
# EXPORT #
##########
ament_export_dependencies(rosidl_default_runtime)
ament_package()
