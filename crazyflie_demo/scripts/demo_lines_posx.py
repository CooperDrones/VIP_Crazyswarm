#!/usr/bin/env python
from demo_cooper import Demo

if __name__ == '__main__':
    demo_pos = Demo(
        [
            # # x, y, z, yaw, sleep
            # [0.35, 0,     0.5, 0, 5],
            # [0.35, -0.5,  0.5, 0, 5],
            # [0.35, 0.5,   0.5, 0, 5],
            # [0.35, -0.5,  0.5, 0, 5],
            # [0.35, 0.5,   0.5, 0, 5],
            # [0.35, -0.5,  0.5, 0, 5],
            # [0.35, 0,     0.5, 0, 0],

            # Sqaure formation
            [0.35, 0,    0.4, 0, 1],
            [0.35, -0.4, 0.4, 0, 1],
            [0.35, -0.4, 0.8, 0, 1],
            [0.35, 0.4,  0.8, 0, 1],
            [0.35, 0.4,  0.4, 0, 1],
            [0.35, 0,    0.4, 0, 1],

            # # Circle formation
            # [0.35, 0,       0.4,   0, 1],
            # [0.35, -0.212,  0.612, 0, 1],
            # [0.35, -0.3,    0.7,   0, 1],
            # [0.35, -0.212,  0.912, 0, 1],
            # [0.35, 0,       1.0,   0, 1],
            # [0.35, 0.212,   0.912, 0, 1],
            # [0.35, 0.3,     0.7,   0, 1],
            # [0.35, 0.212,   0.612, 0, 1],
            # [0.35, 0,       0.4,   0, 1],

            # # Hover at photogenic point
            # [0.35, -0.75, 0.8, 0, 0],
        ]
    , '/crazyflie4')
    demo_pos.run()