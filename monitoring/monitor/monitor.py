#!/usr/bin/env python3
"""Combined production monitor — both Basin and Moon."""

import os
from robot_monitor import RobotConfig, run

BASIN_IP = os.environ.get("BASIN_IP", "192.168.1.158")
BASIN_NS = os.environ.get("BASIN_NAMESPACE", "/Basin")
MOON_IP = os.environ.get("MOON_IP", "192.168.1.227")
MOON_NS = os.environ.get("MOON_NAMESPACE", "/Moon")

if __name__ == "__main__":
    run([
        RobotConfig(name="Basin", ip=BASIN_IP, namespace=BASIN_NS),
        RobotConfig(name="Moon", ip=MOON_IP, namespace=MOON_NS),
    ])
