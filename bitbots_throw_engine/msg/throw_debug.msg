#
# This message is only for debugging / visualizing the throw
#

std_msgs/Header header

# the following is computed to trunk frame
# position of the goals, the resulting positions computed by running FK on the IK results and the actual position from joint states
geometry_msgs/Pose left_hand_goal
geometry_msgs/Pose right_hand_goal
geometry_msgs/Pose trunk_goal
