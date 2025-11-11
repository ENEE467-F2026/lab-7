cmake_minimum_required(VERSION 3.8)
project(ur3e_hande_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GetTargetObjPose.action"
  "msg/ObjectMetaData.msg"
  "msg/ObjectUId.msg"
  "msg/InferenceResult.msg"
  "msg/YoloInference.msg"
  "msg/DetectedSurfaces.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

install(DIRECTORY
  msg action scripts utils config rviz launch
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS scripts/yolo_pc_pose_estimation.py
   DESTINATION lib/${PROJECT_NAME}
 )

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs sensor_msgs)
ament_package()
