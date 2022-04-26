#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
import rospy

if __name__ == '__main__':
    cf3 = CooperativeQuad('crazyflie3')
    cf3.hoverStiff(0.0, 0.30, 0.30, 0.0, 0.05, False) 
    # To change z var, change it in a_cooperative_quad.py