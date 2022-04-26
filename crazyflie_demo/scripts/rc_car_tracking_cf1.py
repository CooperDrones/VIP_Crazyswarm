#!/usr/bin/env python
from rc_car_tracking_1 import Demo

if __name__ == '__main__':
    cf1_rc_car_tracker = Demo(0.0, 0.0, 0.5, '/crazyflie1')
    cf1_rc_car_tracker.run()
