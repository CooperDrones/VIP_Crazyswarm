#!/usr/bin/env python
from safety_check import safety_check

if __name__ == '__main__':
    cf3_rc_car_tracker = safety_check(0.0, 0.0, 0.5, '/crazyflie3')
    cf3_rc_car_tracker.run()
