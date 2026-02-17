#!/usr/bin/env python3
"""Moon production monitor — sole subscriber, no rosbag."""

import os
from robot_monitor import RobotConfig, run

MOON_IP = os.environ.get("MOON_IP", "192.168.1.227")
MOON_NS = os.environ.get("MOON_NAMESPACE", "/Moon")

if __name__ == "__main__":
    run([RobotConfig(name="Moon", ip=MOON_IP, namespace=MOON_NS)])
