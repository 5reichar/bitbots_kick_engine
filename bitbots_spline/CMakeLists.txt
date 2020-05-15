cmake_minimum_required(VERSION 2.8.3)
project(bitbots_spline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  eigen_conversions
  visualization_msgs
  bio_ik
)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCES
  src/visualisation/visual_splines_service.cpp
  src/visualisation/visual_splines_node.cpp
  src/visualisation/visual_splines_material.cpp
  src/utils/combination.cpp
  src/utils/newton_binomial.cpp
  src/utils/polynom.cpp
  src/utils/pose_spline.cpp
  src/utils/spline_container_factory.cpp
  src/utils/trajectory_service.cpp
  src/spline/curve.cpp
  src/spline/spline.cpp
  src/spline/linear_spline.cpp
  src/spline/cubic_spline.cpp
  src/spline/smooth_spline.cpp
  src/spline/beziercurve.cpp
  src/handle/position_handle.cpp
  src/handle/pose_handle.cpp

  include/utils/abstract_engine.h
  include/utils/abstract_stabilizer.h
  include/utils/abstract_ik.h
  include/utils/abstract_visualizer.h
	include/utils/reference_goals.h
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${SOURCES}
)

add_executable(
    VisualSplinesNode src/visualisation/visual_splines_node.cpp
)

target_link_libraries(
    VisualSplinesNode ${PROJECT_NAME} ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
