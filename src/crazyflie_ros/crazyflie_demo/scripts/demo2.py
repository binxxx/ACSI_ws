#!/usr/bin/env python
from demo import Demo

if __name__ == '__main__':
    demo = Demo(
        [
            #x   ,   y,   z, yaw, sleep
            [0.0 , 0.0, 0.8, 0, 3],
            [0.5 , 0.0, 0.8, 0, 3],
            [0.0 , 0.5, 0.8, 0, 3],
            [-0.5 , 0.0, 0.8, 0, 3],
            [0.0 , -0.5, 0.8, 0, 3],
        ]
    )
    demo.run()
