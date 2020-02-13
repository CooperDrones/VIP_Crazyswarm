#!/usr/bin/env python
from a_hover_stiff_multi import CooperativeQuad
import rospy

if __name__ == '__main__':
    cf5 = CooperativeQuad('crazyflie5')
    cf5.hoverStiff(-1.0, 0.0, 0.4, 0.0, 0.1)
    # cf5.land()