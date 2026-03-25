"""
Semantic navigation test scenario

Sends a natural language instruction and waits for the robot to reach the target object.
Success: robot enters goal_radius around any target object.
Failure: timeout or FAILED planner status.
"""
import json
import time
from typing import List, Optional, Tuple

import numpy as np

from .base import Scenario


class SemanticNavScenario(Scenario):
    """Semantic navigation test scenario.

    Usage:
        scenario = SemanticNavScenario(
            instruction="navigate to the factory entrance",
            target_labels=["door", "gate"],
            target_positions=[(25.0, 10.0)],   # optional: known target positions
            goal_radius=2.0,
            max_time=180.0,
        )
    """

    name = "semantic_nav"
    description = "Semantic navigation: send natural language instruction, check arrival near target"

    def __init__(
        self,
        instruction: str,
        target_labels: Optional[List[str]] = None,
        target_positions: Optional[List[Tuple[float, float]]] = None,
        goal_radius: float = 2.0,
        max_time: float = 180.0,
    ):
        super().__init__()
        self.instruction = instruction
        self.target_labels = target_labels or []
        self.target_positions = target_positions or []
        self.goal_radius = goal_radius
        self.max_time = max_time

        self._success = False
        self._failure_reason: str = ""
        self._status_history: List[dict] = []
        self._last_planner_status: str = ""

        # ROS2 subscriptions (lazy init)
        self._status_sub = None
        self._planner_sub = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def setup(self, engine) -> None:
        super().setup(engine)

        # Subscribe to semantic planner status
        self._subscribe_status(engine)

        # Send semantic instruction
        self._send_instruction(engine)

        print(
            f"[SemanticNavScenario] instruction='{self.instruction}' "
            f"targets={self.target_labels} timeout={self.max_time}s"
        )

    def is_complete(self, engine) -> bool:
        if self.is_timeout():
            self._failure_reason = "timeout"
            return True

        if self._success:
            return True

        # Check for planner failure status
        if self._last_planner_status == "FAILED":
            self._failure_reason = "planner_failed"
            return True

        if engine is None:
            return False

        # Check whether robot reached any known target position
        if self.target_positions:
            state = engine.get_robot_state()
            if state is not None:
                pos = state.position
                for tp in self.target_positions:
                    if self._distance_2d(pos, tp) <= self.goal_radius:
                        self._success = True
                        self._metrics["reached_target"] = list(tp)
                        return True

        return False

    def is_success(self, engine) -> bool:
        return self._success

    def get_metrics(self):
        metrics = super().get_metrics()
        metrics["instruction"] = self.instruction
        metrics["target_labels"] = self.target_labels
        metrics["goal_radius_m"] = self.goal_radius
        metrics["planner_status_final"] = self._last_planner_status
        if self._failure_reason:
            metrics["failure_reason"] = self._failure_reason
        return metrics

    # ── Internal ──────────────────────────────────────────────────────────────

    def _send_instruction(self, engine):
        """Publish semantic instruction to /nav/semantic/instruction."""
        try:
            from std_msgs.msg import String

            node = getattr(engine, "_bridge_node", None)
            if node is None:
                return

            pub = node.create_publisher(String, "/nav/semantic/instruction", 10)
            msg = String()
            msg.data = self.instruction
            # Delay 1 second to allow nav stack to become ready
            import threading
            def _delayed():
                time.sleep(1.0)
                pub.publish(msg)
                print(f"[SemanticNavScenario] instruction sent: '{self.instruction}'")
            threading.Thread(target=_delayed, daemon=True).start()
        except Exception as e:
            print(f"[SemanticNavScenario] instruction send failed: {e}")

    def _subscribe_status(self, engine):
        """Subscribe to semantic status and planner status topics."""
        try:
            from std_msgs.msg import String

            node = getattr(engine, "_bridge_node", None)
            if node is None:
                return

            def _semantic_status_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self._status_history.append(data)
                    # Check if semantic planner reports success
                    phase = data.get("phase", "")
                    if phase in ("goal_reached", "succeeded"):
                        self._success = True
                except Exception:
                    pass

            def _planner_status_cb(msg):
                self._last_planner_status = msg.data.strip()

            self._status_sub = node.create_subscription(
                String, "/nav/semantic/status", _semantic_status_cb, 10)
            self._planner_sub = node.create_subscription(
                String, "/nav/planner_status", _planner_status_cb, 10)
        except Exception as e:
            print(f"[SemanticNavScenario] status subscribe failed: {e}")
