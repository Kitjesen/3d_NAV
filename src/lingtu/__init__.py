"""LingTu — import-ready functional packages.

Usage::

    from lingtu import LiDAR, SLAM, Navigator, Camera, Detector, Robot

    # Hardware
    lidar = LiDAR()
    camera = Camera()

    # SLAM
    slam = SLAM(lidar)
    slam.save_map("building_a")

    # Navigation
    nav = Navigator(slam)
    nav.go("体育馆")
    nav.go_to(5.0, 3.0)
    nav.stop()

    # Perception
    detector = Detector(camera)
    objects = detector.detect()

    # Full robot (all-in-one, builds a real blueprint system)
    robot = Robot("sim").start()    # sim/nav/explore/map/dev
    robot.go("体育馆")               # semantic navigation
    robot.follow("person in red")   # follow a described person
    robot.stop_follow()
    robot.save_map("building_a")
    robot.shutdown()
"""

from .camera import Camera
from .detector import Detector
from .lidar import LiDAR
from .navigator import Navigator
from .robot import Robot
from .slam import SLAM

__all__ = ["SLAM", "Camera", "Detector", "LiDAR", "Navigator", "Robot"]
