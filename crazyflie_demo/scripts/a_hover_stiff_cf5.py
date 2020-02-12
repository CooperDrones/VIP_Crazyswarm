#!/usr/bin/env python
from a_hover_stiff import CooperativeQuad
import rospy

if __name__ == '__main__':
    # rospy.init_node('cooperative_quad5', anonymous=True)
    cf5 = CooperativeQuad('crazyflie5')
    cf5.hoverStiff(-1.0, 0.0, 0.4, 0.0, 0.1)
    cf5.land()