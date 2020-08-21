#
# This message is only for debugging / visualizing the throw
#

std_msgs/Header header

# the following is computed to trunk frame
# position of the goals, the resulting positions computed by running FK on the IK results and the actual position from joint states

uint8 percentage_done
uint8 movement_stage
geometry_msgs/Pose left_hand_goal
geometry_msgs/Pose right_hand_goal
geometry_msgs/Pose trunk_goal
geometry_msgs/Pose left_foot_goal
geometry_msgs/Pose right_foot_goal
