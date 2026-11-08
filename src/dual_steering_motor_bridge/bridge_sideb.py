#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Side B entrypoint; runs exactly the SAME MotorBridge class as Side A.
Put bridge_sidea.py in the same directory. Only side and ROS parameters differ.
"""
from bridge_sidea import main

if __name__ == '__main__':
    main('side_b')
