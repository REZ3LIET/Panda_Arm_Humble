cmake_minimum_required(VERSION 3.8)
project(franka_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY
  launch
  world
  DESTINATION share/${PROJECT_NAME})

ament_package()
