# ~/tb4_ws/src/robots/setup.py
from setuptools import setup
import os
from glob import glob

package_name = "robots"
# Path to this package (so dashboard glob works when colcon builds from build dir)
pkg_dir = os.path.dirname(os.path.abspath(__file__))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files (make paths relative to the package dir)
        (
            os.path.join("share", package_name, "launch"),
            [os.path.join(os.path.relpath(os.path.dirname(p), pkg_dir), os.path.basename(p)) for p in glob(os.path.join(pkg_dir, "launch", "*.launch.py"))],
        ),
        # Include config files
        (
            os.path.join("share", package_name, "config"),
            [os.path.relpath(p, pkg_dir) for p in glob(os.path.join(pkg_dir, "config", "*.yaml"))],
        ),
        # Include map files
        (
            os.path.join("share", package_name, "maps"),
            [os.path.relpath(p, pkg_dir) for p in glob(os.path.join(pkg_dir, "maps", "*"))],
        ),
        # Dashboard (Phase 1 web UI)
        (
            os.path.join("share", package_name, "dashboard"),
            # Support dashboard files located either in 'dashboard/' or 'dashboard/dashboard/'
            [os.path.relpath(p, pkg_dir) for p in glob(os.path.join(pkg_dir, "dashboard", "*.html"))]
            + [os.path.relpath(p, pkg_dir) for p in glob(os.path.join(pkg_dir, "dashboard", "*.css"))]
            + [os.path.relpath(p, pkg_dir) for p in glob(os.path.join(pkg_dir, "dashboard", "dashboard", "*.html"))]
            + [os.path.relpath(p, pkg_dir) for p in glob(os.path.join(pkg_dir, "dashboard", "dashboard", "*.css"))],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Multi-robot localization and navigation package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "laserscan_frame_corrector = robots.laserscan_frame_corrector:main",
            "scan_frame_corrector = robots.scan_frame_corrector:main",
            "tf_relay = robots.tf_relay_node:main",
            "odom_to_tf_broadcaster = robots.odom_to_tf_broadcaster:main",
            "odom_relay = robots.odom_relay:main",
            "clock_swing_controller = robots.clock_swing_controller_improved:main",
            "clock_swing_controller_exhibit = robots.clock_swing_controller_exhibit:main",
            "exhibition_master_controller = robots.exhibition_master_controller:main",
            "exhibition_final_controller = robots.exhibition_final_controller:main",
            "waypoint_recorder = robots.waypoint_recorder:main",
            "phase1_controller = robots.phase1_controller:main",
            "phase2_circle_controller = robots.phase2_circle_controller:main",
            "launch_dashboard = robots.launch_dashboard:main",
            "wait_for_topic = robots.wait_for_topic:main",
        ],
    },
)