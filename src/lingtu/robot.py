"""Robot — all-in-one, one import does everything."""

from __future__ import annotations

import logging
import time

logger = logging.getLogger(__name__)


class Robot:
    """Complete robot interface — one object, full capability.

    Usage::

        robot = Robot()
        robot.start()

        # Navigation
        robot.go("体育馆")
        robot.go_to(5.0, 3.0)
        robot.stop_motion()

        # Mapping
        robot.save_map("building_a")
        robot.use_map("building_a")

        # Perception
        objects = robot.detect()
        robot.find("red chair")

        # Follow
        robot.follow("person in red")

        robot.shutdown()
    """

    def __init__(self, mode: str = "nav"):
        """
        Args:
            mode: "nav" (navigation with existing map),
                  "map" (build new map),
                  "explore" (navigate without map)
        """
        self._mode = mode
        self._lidar = None
        self._camera = None
        self._slam = None
        self._nav = None
        self._detector = None
        self._started = False

    def start(self) -> Robot:
        """Start all subsystems."""
        if self._started:
            return self

        from .camera import Camera
        from .detector import Detector
        from .lidar import LiDAR
        from .navigator import Navigator
        from .slam import SLAM

        # Determine SLAM mode
        slam_mode = {
            "nav": "localizer",
            "map": "fastlio2",
            "explore": "fastlio2",
        }.get(self._mode, "bridge")

        self._lidar = LiDAR().start()
        self._camera = Camera().start()
        self._slam = SLAM(self._lidar, mode=slam_mode).start()
        self._nav = Navigator(self._slam).start()
        self._detector = Detector(self._camera).start()

        self._started = True
        logger.info("Robot started (mode=%s)", self._mode)
        return self

    def shutdown(self) -> None:
        """Stop all subsystems."""
        for sub in [self._nav, self._detector, self._slam, self._camera, self._lidar]:
            if sub:
                try:
                    sub.stop()
                except Exception as e:
                    logger.debug("Robot: subsystem cleanup error: %s", e)
        self._started = False

    # Navigation
    def go(self, instruction: str) -> str:
        return self._nav.go(instruction) if self._nav else "Not started"

    def go_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        return self._nav.go_to(x, y, yaw) if self._nav else "Not started"

    def stop_motion(self) -> str:
        return self._nav.stop_motion() if self._nav else "Not started"

    def status(self) -> str:
        return self._nav.status() if self._nav else "NOT_STARTED"

    # Mapping
    def save_map(self, name: str) -> bool:
        return self._slam.save_map(name) if self._slam else False

    def use_map(self, name: str) -> None:
        if self._nav:
            self._nav.use_map(name)

    # Perception
    def detect(self):
        return self._detector.detect() if self._detector else []

    def find(self, label: str):
        return self._detector.find(label) if self._detector else None

    # Pose
    def get_pose(self):
        return self._slam.get_pose() if self._slam else None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.shutdown()

    def __repr__(self):
        return "Robot(mode=%s, status=%s)" % (self._mode, self.status())
