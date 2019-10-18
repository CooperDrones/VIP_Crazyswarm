#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

# class Navigator:
#     def __init__(self):
#         self.sub = rospy.Subscriber("vicon/crazyflie2/crazyflie2", TransformStamped, self.callback)
#         self.msg = Hover()
#         self.twist_msg = Twist()
#         self.duration_msg = Duration()
#         # self.duration_msg.data.secs = 10f
#         self.hz = 100
#         self.rate = rospy.Rate(self.hz)

#         # rospy.wait_for_service('/crazyflie/takeoff')
#         # self.takeoff_command = rospy.ServiceProxy('/crazyflie/takeoff', Takeoff)

#         self.pub = rospy.Publisher("crazyflie2/cmd_hover", Hover, queue_size=0)
#         self.pub_twist = rospy.Publisher("crazyflie2/cmd_vel", Twist, queue_size=0)

#         rospy.wait_for_service('/vicon/grab_vicon_pose')
#         self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    # x = rospy.get_param("~x")
    # y = rospy.get_param("~y")
    # z = rospy.get_param("~z")
    x = 0; y = 0; z = 0.5

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    msg2 = PoseStamped()
    msg2.header.seq = 0
    msg2.header.stamp = rospy.Time.now()
    msg2.header.frame_id = worldFrame
    msg2.pose.position.x = x 
    msg2.pose.position.y = y + 0.5
    msg2.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg2.pose.orientation.x = quaternion[0]
    msg2.pose.orientation.y = quaternion[1]
    msg2.pose.orientation.z = quaternion[2]
    msg2.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    counter = 0
    while not rospy.is_shutdown():
        if counter >= 1500:
            msg = msg2
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
        print(counter)
        counter += 1
