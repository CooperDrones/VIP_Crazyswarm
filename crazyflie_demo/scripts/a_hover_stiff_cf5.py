#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
import rospy

if __name__ == '__main__':
    cf5 = CooperativeQuad('crazyflie5')
    cf5.hoverStiff(-1.0, 0.0, 0.35, 0.0, 0.05, False)