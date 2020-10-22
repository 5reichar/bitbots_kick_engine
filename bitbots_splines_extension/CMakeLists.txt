cmake_minimum_required(VERSION 2.8.3)
project(bitbots_splines_extension)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  tf2
  geometry_msgs
  visualization_msgs
  bitbots_splines
)

find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
  src/visualisation/visual_splines_service.cpp
  src/visualisation/visual_splines_node.cpp
  src/visualisation/visual_splines_material.cpp
  src/spline/curve.cpp
  src/spline/spline_base.cpp
  src/spline/linear_spline.cpp
  src/spline/cubic_spline.cpp
  src/spline/quintic_spline.cpp
  src/spline/beziercurve.cpp
  src/handle/position_handle.cpp
  src/handle/pose_handle.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(VisualSplinesNode src/visualisation/visual_splines_node.cpp)

target_link_libraries(VisualSplinesNode ${PROJECT_NAME} ${catkin_LIBRARIES})