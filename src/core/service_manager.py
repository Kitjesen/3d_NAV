"""ServiceManager — start/stop systemd services on demand.

Industrial pattern: only run what you need, release when done.

Usage:
    svc = ServiceManager()
    svc.start("nav-lidar", "nav-slam")   # pull up when needed
    svc.stop("nav-lidar", "nav-slam")    # release when done
    svc.ensure("nav-lidar")              # start only if not running
"""

from __future__ import annotations

import logging
import subprocess
import time
from typing import List

logger = logging.getLogger(__name__)


class ServiceManager:
    """Manage systemd services — start on demand, stop on release."""

    def __init__(self):
        self._started: List[str] = []  # services we started (we'll stop them)

    def is_running(self, service: str) -> bool:
        """Check if a systemd service is active."""
        try:
            result = subprocess.run(
                ["systemctl", "is-active", "--quiet", service],
                timeout=3,
            )
            return result.returncode == 0
        except Exception:
            return False

    def start(self, *services: str) -> List[str]:
        """Start services. Returns list of actually started services."""
        started = []
        for svc in services:
            if self.is_running(svc):
                logger.debug("Service %s already running", svc)
                continue
            try:
                subprocess.run(
                    ["sudo", "systemctl", "start", svc],
                    timeout=10, check=True,
                    capture_output=True,
                )
                self._started.append(svc)
                started.append(svc)
                logger.info("Started service: %s", svc)
            except subprocess.CalledProcessError as e:
                logger.error("Failed to start %s: %s", svc, e.stderr.decode()[:100])
            except Exception as e:
                logger.error("Failed to start %s: %s", svc, e)
        return started

    def stop(self, *services: str) -> None:
        """Stop services."""
        for svc in services:
            try:
                subprocess.run(
                    ["sudo", "systemctl", "stop", svc],
                    timeout=10, capture_output=True,
                )
                if svc in self._started:
                    self._started.remove(svc)
                logger.info("Stopped service: %s", svc)
            except Exception as e:
                logger.warning("Failed to stop %s: %s", svc, e)

    def ensure(self, *services: str) -> None:
        """Ensure services are running (start if not)."""
        self.start(*services)

    def stop_all_started(self) -> None:
        """Stop all services that we started (cleanup)."""
        for svc in list(self._started):
            self.stop(svc)

    def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
        """Wait until all services are active."""
        deadline = time.time() + timeout
        pending = list(services)
        while pending and time.time() < deadline:
            pending = [s for s in pending if not self.is_running(s)]
            if pending:
                time.sleep(0.5)
        if pending:
            logger.warning("Services not ready after %.0fs: %s", timeout, pending)
            return False
        return True


# Singleton
_manager = ServiceManager()


def get_service_manager() -> ServiceManager:
    return _manager


# Service groups — what each mode needs
SERVICES_SLAM = ["nav-lidar", "nav-slam"]
SERVICES_CAMERA = ["orbbec-camera"]
SERVICES_ALL = SERVICES_SLAM + SERVICES_CAMERA
