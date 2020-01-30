#!/usr/bin/env python
from a_hover_stiff import CooperativeQuad
import rospy

if __name__ == '__main__':
    # rospy.init_node('cooperative_quad4', anonymous=True)
    cf4 = CooperativeQuad('crazyflie4')
    cf4.hoverStiff(-0.5, 0.0, 0.4, 0.0, 0.1)
    cf4.land()