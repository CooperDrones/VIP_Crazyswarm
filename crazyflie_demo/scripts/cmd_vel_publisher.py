#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Vector3,PoseStamped
from tf import TransformListener

rospy.init_node('test')
pub = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size=0)
rate = rospy.Rate(100)

t = Twist() # instatiate a command 
t.linear = Vector3(0, 0, 10000)
t.angular = Vector3(0, 0, 0)

# goal = PoseStamped()
# goal.pose.position.x = 0
# goal.pose.position.y = 0
# goal.pose.position.z = 0.5
# quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
# goal.pose.orientation.x = quaternion[0]
# goal.pose.orientation.y = quaternion[1]
# goal.pose.orientation.z = quaternion[2]
# goal.pose.orientation.w = quaternion[3]

while not rospy.is_shutdown():
    pub.publish(t)
    # pubGoal.publish(goal)
    rate.sleep()