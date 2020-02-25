#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
import rospy

if __name__ == '__main__':
    cf4 = CooperativeQuad('crazyflie4')
    cf4.hoverStiff(-0.5, 0.0, 0.35, 0.0, 0.05, False)