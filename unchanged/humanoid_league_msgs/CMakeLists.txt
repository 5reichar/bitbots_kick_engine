cmake_minimum_required(VERSION 2.8.3)
project(humanoid_league_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    AdditionalServoData.msg
    Animation.msg
    BallInImage.msg
    BallRelative.msg
    BallsInImage.msg
    BarInImage.msg
    GameState.msg
    GoalInImage.msg
    GoalPartsInImage.msg
    GoalRelative.msg
    HeadMode.msg
    ImageWithRegionOfInterest.msg
    LineCircleInImage.msg
    LineCircleRelative.msg
    LineIntersectionInImage.msg
    LineIntersectionRelative.msg
    LineSegmentInImage.msg
    LineSegmentRelative.msg
    LineInformationInImage.msg
    LineInformationRelative.msg
    Model.msg
    ObstacleInImage.msg
    ObstaclesInImage.msg
    ObstacleRelative.msg
    ObstaclesRelative.msg
    PixelRelative.msg
    PixelsRelative.msg
    Position2D.msg
    PostInImage.msg
    RobotControlState.msg
    Speak.msg
    Strategy.msg
    TeamData.msg
    VisualCompassRotation.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PlayAnimation.action
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   trajectory_msgs
   actionlib_msgs
 )

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   std_msgs
   geometry_msgs
   sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
