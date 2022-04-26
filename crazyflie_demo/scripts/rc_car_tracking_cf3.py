#!/usr/bin/env python
from rc_car_tracking import Demo

if __name__ == '__main__':
    cf3_rc_car_tracker = Demo(0.0, 0.0, 0.5, '/crazyflie3')
    cf3_rc_car_tracker.run()
