#!/usr/bin/env python
from a_hover_stiff_multi import CooperativeQuad
import rospy

if __name__ == '__main__':
    cf4 = CooperativeQuad("crazyflie4")
    # cf4.listener()
    cf4.hoverStiff(-0.5, 0.0, 0.4, 0.0, 0.1)
    # cf4.land()