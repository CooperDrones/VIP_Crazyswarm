#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
from crazyflie_driver.srv import GoTo
from crazyflie_driver.srv import Takeoff
from crazyflie_driver.srv import Land

class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)

        self.pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.msg = Hover()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        self.msg.yawrate = 0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()


    # determine direction of speed based on distance
    def getSpeed(self, distance):
        if distance > 0:
            return 0.1
        elif distance < 0:
            return -0.1
        else:
            return 0

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    #take off to z distance
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

    # land from last zDistance
    # def land (self):
    #     # get last height
    #     zDistance = self.msg.zDistance

    #     while not rospy.is_shutdown():
    #         while zDistance > 0:
    #             self.msg.vx = 0.0
    #             self.msg.vy = 0.0
    #             self.msg.yawrate = 0.0
    #             self.msg.zDistance = zDistance
    #             self.msg.header.seq += 1
    #             self.msg.header.stamp = rospy.Time.now()
    #             self.pub.publish(self.msg)
    #             self.rate.sleep()
    #             zDistance -= 0.2
    #     self.stop_pub.publish(self.stop_msg)

def handler(cf):
    print("This is supposed to take off")
    cf.takeOff(0.35)
    # cf.TakeOff(0,0.35,10)
    cf.GoTo(0 0 [0 1 0.35] 0 10)
    cf.GoTo(0 0 [0 -1 0.35] 0 10)
    # cf.Land(0,0.35,10)

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)

    cf1 = Crazyflie("cf1")
    # cf2 = Crazyflie("cf2")

    t1 = Thread(target=handler, args=(cf1,))
    # t2 = Thread(target=handler, args=(cf2,))
    t1.start()
    # t2.start()