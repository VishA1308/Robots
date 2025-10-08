cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CleaningTask.action"
)

# Install Python scripts
install(PROGRAMS
  scripts/cleaning_server.py
  scripts/cleaning_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install bin wrappers (ОСНОВНЫЕ ИСПОЛНЯЕМЫЕ ФАЙЛЫ)
install(PROGRAMS
  bin/cleaning_action_server
  bin/cleaning_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclpy)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(turtlesim)
ament_export_dependencies(action_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
