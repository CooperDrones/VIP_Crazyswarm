#!/usr/bin/env python
from pencil_tracking import Demo

if __name__ == '__main__':
    cf3_pencil_tracker = Demo(0.25, 0.0, '/crazyflie3')
    cf3_pencil_tracker.run()
