#!/usr/bin/env python3

import pygame
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray


pygame.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()

rospy.init_node('Joystick_publisher', anonymous=True)
destination_publisher = rospy.Publisher('Destination', Float64MultiArray, queue_size=1)

pub_destination = Float64MultiArray()


while True:
    pygame.event.pump()
    x = pygame.joystick.Joystick(0).get_axis(0)/100
    y = pygame.joystick.Joystick(0).get_axis(1)/100
    z = -pygame.joystick.Joystick(0).get_axis(4)/100
    pub_destination.data = np.array([x,y,z])
    destination_publisher.publish(pub_destination)
    rospy.sleep(0.01)