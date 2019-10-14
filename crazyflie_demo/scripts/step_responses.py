#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3,TransformStamped # twist used in cmd_vel
from crazyflie_driver.msg import Hover # used in cmd_hover commands vel, yaw rate, and hover height
from crazyflie_driver.srv import Takeoff
from std_msgs.msg import Duration

class Tester:
    def __init__(self):

        # worldFrame = rospy.get_param("~worldFrame", "/world")      
        self.sub = rospy.Subscriber("/vicon/crazyflie1/crazyflie1", TransformStamped, self.callback)
        self.msg = Hover()
        self.twist_msg = Twist()
        self.duration_msg = Duration()
        # self.duration_msg.data.secs = 10f
        self.hz = 100
        self.rate = rospy.Rate(self.hz)

        # rospy.wait_for_service('/crazyflie/takeoff')
        # self.takeoff_command = rospy.ServiceProxy('/crazyflie/takeoff', Takeoff)

        self.pub = rospy.Publisher("crazyflie1/cmd_hover", Hover, queue_size=0)
        self.pub_twist = rospy.Publisher("crazyflie1/cmd_vel", Twist, queue_size=0)

    def callback(self, data):
        self.current_state = data

    def takeOff(self, zDistance):
        time_range = 1 + int(10*zDistance/0.4)
        while not rospy.is_shutdown():
            for y in range(time_range):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = y / 25.0
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            for y in range(20):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            break
    
    def commandAction(self, roll, pitch, thrust, yaw_rate, action_time):
        self.twist_msg.linear = Vector3(roll, pitch, thrust)
        self.twist_msg.angular = Vector3(0, 0, yaw_rate)

        data_points = int(self.hz*action_time)
        for i in range(data_points):
            self.pub_twist.publish(self.twist_msg)
            self.rate.sleep()

    def commandThrust(self, thrust): # tested and works
        self.twist_msg.linear = Vector3(0, 0, thrust)
        self.twist_msg.angular = Vector3(0, 0, 0)

        while not rospy.is_shutdown():
            self.pub_twist.publish(self.twist_msg)
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



    test1 = Tester()
    # test1.takeOff(0.4) # Using hover msg which uses zdistance
    # test1.commandThrust(10000)

    roll = 0
    pitch = 0
    thrust = 10000
    yaw_rate = 0
    time_seconds = 1
    test1.commandAction(roll, pitch, thrust, yaw_rate, time_seconds) 
    # test1.commandAction(10)

    # pub = rospy.Publisher('crazyflie1/cmd_vel', Twist, queue_size=0)
    # rate = rospy.Rate(100)
    # t = Twist() # instatiate a command 
    # t.linear = Vector3(0, 0, 40000) # ~40000 thrust is enough to takeoff
    # t.angular = Vector3(0, 0, 0)
    # for i in range(30): # publish for 1/3 of a second
    # # while not rospy.is_shutdown():
    #     pub.publish(t)
    #     rate.sleep()