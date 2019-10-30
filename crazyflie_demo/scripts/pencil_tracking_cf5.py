#!/usr/bin/env python
from pencil_tracking import Demo

if __name__ == '__main__':
    cf5_pencil_tracker = Demo(0.0, 0.25, '/crazyflie4')
    cf5_pencil_tracker.run()