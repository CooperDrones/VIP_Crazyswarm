#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3,TransformStamped
from crazyflie_driver.srv import Takeoff
from std_msgs.msg import Duration

class Tester:
    def __init__(self):

        # worldFrame = rospy.get_param("~worldFrame", "/world")      
        self.sub = rospy.Subscriber("/vicon/crazyflie1/crazyflie1", TransformStamped, self.callback)
        self.twist_msg = Twist()
        self.duration_msg = Duration()
        # self.duration_msg.data.secs = 10f
        self.rate = rospy.Rate(10)

        # rospy.wait_for_service('/crazyflie/takeoff')
        # self.takeoff_command = rospy.ServiceProxy('/crazyflie/takeoff', Takeoff)

        self.pub = rospy.Publisher("crazyflie1/cmd_vel", Twist, queue_size=0)

    def callback(self, data):
        self.current_state = data

    def takeOff(self, z):
        # self.takeoff_command(1, 0, self.duration_msg)
        None
    
    def commandThrust(self, thrust):
        self.twist_msg.linear = Vector3(0, 0, thrust)
        self.twist_msg.angular = Vector3(0, 0, 0)
        # self.twist_msg.linear.z = thrust

        self.pub.publish(self.twist_msg)
        self.rate.sleep()


# run takeoff command but with twist message instead of hover message
# def takeOff(pub):
#     # msg = Twist()
#     rate = rospy.Rate(100)
#     # time_range = 1 + int(10*z/0.4)
#     t = Twist() # instatiate a command 
#     t.linear = Vector3(0, 0, 10000)
#     t.angular = Vector3(0, 0, 0)

#     while not rospy.is_shutdown():
#         pub.publish(t)
#         rate.sleep()
        # for y in range(time_range):
        #     msg.linear.x = 0.0
        #     msg.linear.y = 0.0
        #     msg.angular.z = 0.0
        #     msg.linear.z = y / 25.0
        #     # msg.header.seq += 1
        #     # msg.header.stamp = rospy.Time.now()
        #     pub.publish(msg)
        #     rate.sleep()
        # for y in range(20):
        #     msg.linear.x = 0.0
        #     msg.linear.y = 0.0
        #     msg.angular.z = 0.0
        #     msg.linear.z = z
        #     # msg.header.seq += 1
        #     # msg.header.stamp = rospy.Time.now()
        #     pub.publish(msg)
        #     rate.sleep()
        # break

# def hover(hover_duration_s):
#     pub = rospy.Publisher('crazyflie1/cmd_vel', Twist, queue_size=0)
#     hz = 1000
#     rate = rospy.Rate(hz)

#     # number of messages to send 

#     t = Twist() # instatiate a command 
#     t.linear = Vector3(0, 0, 40000)
#     t.angular = Vector3(0, 0, 0)

#     for i in range()
#         pub.publish(t)
#         rate.sleep()



if __name__ == "__main__":
    rospy.init_node('test')

    pub = rospy.Publisher('crazyflie1/cmd_vel', Twist, queue_size=0)
    rate = rospy.Rate(100)

    # test1 = Tester()
    # test1.takeOff(0.5)
    # test1.commandThrust(10000)

    t = Twist() # instatiate a command 
    t.linear = Vector3(0, 0, 10000)
    t.angular = Vector3(0, 0, 0)

    while not rospy.is_shutdown():
        pub.publish(t)
        rate.sleep()