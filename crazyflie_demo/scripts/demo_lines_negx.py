#!/usr/bin/env python
from demo_cooper import Demo

if __name__ == '__main__':
    demo_neg = Demo(
        [
            # x, y, z, yaw, sleep
            [-0.35, 0,    0.5, 0, 5],
            [-0.35, 0.5,  0.5, 0, 5],
            [-0.35, -0.5, 0.5, 0, 5],
            [-0.35, 0.5,  0.5, 0, 5],
            [-0.35, -0.5, 0.5, 0, 5],
            [-0.35, 0.5,  0.5, 0, 5],
            [-0.35, 0,    0.5, 0, 0],
        ]
    , '/crazyflie3')
    demo_neg.run()