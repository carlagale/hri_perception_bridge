cmake_minimum_required(VERSION 3.8)
project(hri_perception_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

ament_index_register_resource("dnn_models.emotion_recognition" CONTENT "models")


ament_package()