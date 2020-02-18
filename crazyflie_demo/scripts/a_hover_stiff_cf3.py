#!/usr/bin/env python
from a_hover_stiff_multi import CooperativeQuad
import rospy

if __name__ == '__main__':
    cf3 = CooperativeQuad('crazyflie3')
    cf3.hoverStiff(0.0, 0.0, 0.3, 0.0, 0.1)
    cf3.land()