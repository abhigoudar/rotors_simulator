cmake_minimum_required(VERSION 3.5)
project(rotors_description)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
install(
    DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME})
install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME})

    ament_package()