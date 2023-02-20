cmake_minimum_required(VERSION 3.5)
project(rotors_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

set(DEPENDENCIES
   rclcpp
   geometry_msgs
   nav_msgs
   mav_msgs
   sensor_msgs
   trajectory_msgs
)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

add_library(lee_position_controller
  src/library/lee_position_controller.cpp
)
target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
ament_target_dependencies(lee_position_controller ${DEPENDENCIES})

add_library(roll_pitch_yawrate_thrust_controller
  src/library/roll_pitch_yawrate_thrust_controller.cpp
)
target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
ament_target_dependencies(roll_pitch_yawrate_thrust_controller ${DEPENDENCIES})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
ament_target_dependencies(lee_position_controller_node ${DEPENDENCIES})
target_link_libraries(lee_position_controller_node
  lee_position_controller)


# target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
# add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})


# add_executable(roll_pitch_yawrate_thrust_controller_node
#   src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
# add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(roll_pitch_yawrate_thrust_controller_node
#   roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

# install(TARGETS lee_position_controller roll_pitch_yawrate_thrust_controller
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(TARGETS lee_position_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lee_position_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(eigen3_cmake_module ${DEPENDENCIES})
ament_export_include_directories(include)
ament_package()
