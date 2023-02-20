cmake_minimum_required(VERSION 3.5)
project(rotors_gazebo)

add_definitions(-std=c++11)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)
find_package(Python3 COMPONENTS Interpreter)

if(${gazebo_VERSION_MAJOR} GREATER 2)
    message(STATUS "Building iris.sdf.")
    
    set(enable_mavlink_interface "false")
    set(enable_ground_truth "false")
    set(enable_logging "false")
    set(enable_camera "false")
    set(enable_wind "false")
    set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
    set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
    
    # Creates shell commands to generate .sdf file
    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND ${PYTHON_EXECUTABLE} ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir}
      COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
      DEPENDS ${rotors_description_dir}/urdf/iris.xacro
      DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
      DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
      )
    add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)
else()
  message(STATUS "Gazebo version is less than 3, cannot generate iris.sdf (gz tool not available).")
endif()

if(NO_ROS)
  message(STATUS "NO_ROS = true, not building waypoint publisher or hovering example, returning now.")
  return()
endif()

find_package(ament_cmake REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

set(DEPENDENCIES
    gazebo_msgs
    geometry_msgs
    mav_msgs
    rclcpp
    sensor_msgs
    std_srvs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# add_executable(waypoint_publisher src/waypoint_publisher.cpp)
# target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
# ament_target_dependencis(waypoint_publisher ${DEPENDENCIES})

# add_executable(waypoint_publisher_file src/waypoint_publisher_file.cpp)
# target_link_libraries(waypoint_publisher_file ${catkin_LIBRARIES})
# ament_target_dependencis(waypoint_publisher_file ${DEPENDENCIES})

add_executable(hovering_example src/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
ament_target_dependencies(hovering_example ${DEPENDENCIES})

foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}/)
endforeach(dir)

install(TARGETS hovering_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(eigen3_cmake_module ${DEPENDENCIES})
ament_package()