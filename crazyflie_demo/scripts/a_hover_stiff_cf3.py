#!/usr/bin/env python
from a_hover_stiff import CooperativeQuad
import rospy

if __name__ == '__main__':
    # rospy.init_node('cooperative_quad3', anonymous=True)
    cf3 = CooperativeQuad('crazyflie3')
    cf3.hoverStiff(0.0, 0.0, 0.4, 0.0, 0.1)
    cf3.land()