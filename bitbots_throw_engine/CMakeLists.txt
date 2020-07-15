cmake_minimum_required(VERSION 2.8.3)
project(bitbots_throw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  tf2
  bio_ik
  bitbots_splines
  bitbots_splines_extension
  humanoid_league_msgs
  bitbots_msgs
  visualization_msgs
)

find_package(PkgConfig REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  throw_action.msg
  throw_debug.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/throw_params.cfg
  cfg/throw_engine_params.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(
  SOURCES
  src/engine/throw_engine.cpp
  src/ros_interface/throw_node.cpp
  src/ros_interface/publisher/ros_publisher_facade.cpp
  src/ros_interface/publisher/controler_command_publisher.cpp
  src/ros_interface/publisher/odometry_publisher.cpp
  src/ros_interface/publisher/support_publisher.cpp
  src/ros_interface/publisher/debug_publisher.cpp
  src/throws/throw_factory.cpp
  src/throws/throw_curves/throw_curve.cpp
  src/throws/throw_curves/beziercurve_throw.cpp
  src/throws/throw_curves/linear_spline_throw.cpp
  src/throws/throw_curves/cubic_spline_throw.cpp
  src/throws/throw_curves/smooth_spline_throw.cpp
  src/throws/throw_curves/testing_throw.cpp
  src/utility/throw_ik.cpp
  src/utility/throw_stabilizer.cpp)

add_executable(ThrowNode ${SOURCES})

add_dependencies(ThrowNode ${PROJECT_NAME}_gencfg)

target_link_libraries(ThrowNode ${catkin_LIBRARIES})