#!/usr/bin/env python3
"""Basin production monitor — sole subscriber, no rosbag."""

import os
from robot_monitor import RobotConfig, run

BASIN_IP = os.environ.get("BASIN_IP", "192.168.1.158")
BASIN_NS = os.environ.get("BASIN_NAMESPACE", "/Basin")

if __name__ == "__main__":
    run([RobotConfig(name="Basin", ip=BASIN_IP, namespace=BASIN_NS)])
