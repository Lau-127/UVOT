cmake_minimum_required(VERSION 3.8)
project(uvot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(uvot_mecanum_kinematic_system SHARED
  src/uvot_mecanum_kinematic_system.cpp
)

target_compile_features(
  uvot_mecanum_kinematic_system
  PUBLIC cxx_std_17
)

target_link_libraries(
  uvot_mecanum_kinematic_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

install(
  TARGETS uvot_mecanum_kinematic_system
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    launch
    worlds
    config
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
