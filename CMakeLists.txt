cmake_minimum_required(VERSION 3.8)
project(hri_perception_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(hri REQUIRED)
find_package(hri_msgs REQUIRED)

include_directories(${hri_INCLUDE_DIRS} ${hri_DIR}/../../../include/hri)

add_executable(pose_tiago_node src/pose_tiago_node.cpp)
ament_target_dependencies(pose_tiago_node
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
  control_msgs
  ament_index_cpp
)
target_link_libraries(pose_tiago_node yaml-cpp)

add_executable(kinesics_analyzer_node src/kinesics_analyzer_node.cpp)
ament_target_dependencies(kinesics_analyzer_node
  rclcpp
  std_msgs
  hri
  hri_msgs
)

install(TARGETS
  pose_tiago_node
  kinesics_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar directorios en el espacio share para hacerlo portable
install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_index_register_resource("dnn_models.emotion_recognition" CONTENT "models/emotion-ferplus-8.onnx")
ament_package()