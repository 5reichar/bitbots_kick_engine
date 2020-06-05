#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from bitbots_throw.msg import throw_action

rospy.init_node("throw_animation_script")
throw_publisher = rospy.Publisher("/throw", throw_action, queue_size=1)
rospy.sleep(1)

throw_ball  = throw_action()

throw_ball.ball_position = Vector3()
throw_ball.ball_position.x = 0.3
throw_ball.ball_position.y = 0.0
throw_ball.ball_position.z = 0.0

throw_ball.throw_target = Vector3()
throw_ball.throw_target.x = 2.0
throw_ball.throw_target.y = 2.0
throw_ball.throw_target.z = 0.0

throw_publisher.publish(throw_ball)
rospy.sleep(3)
rospy.loginfo("Your throw has been animated")