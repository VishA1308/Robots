cmake_minimum_required(VERSION 3.5)
project(action_cleaning_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CleaningTask.action"
)

# Install Python executables
install(PROGRAMS
  scripts/cleaning_action_server.py
  scripts/cleaning_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install action files
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml and other necessary files
install(
  FILES package.xml
  DESTINATION share/ament_index/resource_index/packages
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
