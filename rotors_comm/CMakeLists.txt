cmake_minimum_required(VERSION 3.5)
project(rotors_comm)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

# Declare the service files to be built
set(srv_files
  "srv/Octomap.srv"
  "srv/RecordRosbag.srv"
)

set(msg_files
  "msg/WindSpeed.msg"
)

set(DEPENDENCIES
  std_msgs
  geometry_msgs
  octomap_msgs)

  rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${DEPENDENCIES}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()