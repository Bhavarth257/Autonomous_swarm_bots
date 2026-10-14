cmake_minimum_required(VERSION 3.8)
project(rescue_hexabot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

# Gazebo system plugin: IK tripod walking gait for the 18-DOF hexapod.
add_library(RescueHexWalk SHARED src/HexWalk.cc)
target_link_libraries(RescueHexWalk
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8)
install(TARGETS RescueHexWalk LIBRARY DESTINATION lib)

# Gazebo system plugin: drive, rocker differential and levelling for the wheel-legged robot.
add_library(RescueWheelLegDrive SHARED src/WheelLegDrive.cc)
target_link_libraries(RescueWheelLegDrive
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8)
install(TARGETS RescueWheelLegDrive LIBRARY DESTINATION lib)

install(PROGRAMS scripts/hexabot_odom DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Sourcing the workspace puts the plugins on Gazebo's search path and selects the
# localhost FastDDS profile (config/fastdds_local.xml) unless one is already set.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/rescue_hexabot.dsv.in")

ament_package()
