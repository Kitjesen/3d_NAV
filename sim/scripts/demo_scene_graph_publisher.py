#!/usr/bin/env python3
"""
Demo 场景图 Stub 发布器 — 模拟 semantic_perception 输出

在没有摄像头或 ML 模型的情况下, 以 1Hz 发布预定义场景图到
/nav/semantic/scene_graph, 让 semantic_planner 可以接收并处理指令。

Usage:
    python3 sim/scripts/demo_scene_graph_publisher.py
    # or with custom scenario
    python3 sim/scripts/demo_scene_graph_publisher.py --scenario office
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ── 场景定义 ──────────────────────────────────────────────

SCENARIOS = {
    "factory": {
        "objects": [
            {"id": "obj_001", "label": "大门", "position": [0.0, 0.0, 0.0],
             "score": 0.96, "detection_count": 12},
            {"id": "obj_002", "label": "门", "position": [3.0, 1.0, 0.0],
             "score": 0.92, "detection_count": 8},
            {"id": "obj_003", "label": "走廊", "position": [4.0, 1.0, 0.0],
             "score": 0.90, "detection_count": 6},
            {"id": "obj_004", "label": "桌子", "position": [5.0, 3.0, 0.0],
             "score": 0.88, "detection_count": 5},
            {"id": "obj_005", "label": "椅子", "position": [5.5, 3.2, 0.0],
             "score": 0.85, "detection_count": 4},
            {"id": "obj_006", "label": "电脑", "position": [5.0, 3.0, 0.7],
             "score": 0.90, "detection_count": 6},
            {"id": "obj_007", "label": "办公室", "position": [5.2, 3.1, 0.0],
             "score": 0.91, "detection_count": 7},
            {"id": "obj_008", "label": "灭火器", "position": [8.0, 5.0, 0.0],
             "score": 0.87, "detection_count": 3},
            {"id": "obj_009", "label": "传送带", "position": [10.0, -2.0, 0.0],
             "score": 0.95, "detection_count": 10},
            {"id": "obj_010", "label": "工厂车间", "position": [10.0, -1.0, 0.0],
             "score": 0.93, "detection_count": 8},
            {"id": "obj_011", "label": "控制台", "position": [12.0, 0.0, 0.0],
             "score": 0.91, "detection_count": 6},
            {"id": "obj_012", "label": "控制室", "position": [12.0, 0.5, 0.0],
             "score": 0.90, "detection_count": 5},
            {"id": "obj_013", "label": "楼梯", "position": [15.0, 2.0, 0.0],
             "score": 0.93, "detection_count": 5},
            {"id": "obj_014", "label": "楼梯间", "position": [15.0, 2.5, 0.0],
             "score": 0.89, "detection_count": 4},
            {"id": "obj_015", "label": "货架", "position": [18.0, -3.0, 0.0],
             "score": 0.89, "detection_count": 4},
            {"id": "obj_016", "label": "仓储区", "position": [18.0, -2.5, 0.0],
             "score": 0.88, "detection_count": 3},
            {"id": "obj_017", "label": "设备间", "position": [20.0, 1.0, 0.0],
             "score": 0.86, "detection_count": 3},
            {"id": "obj_018", "label": "消防栓", "position": [7.0, 0.0, 0.0],
             "score": 0.85, "detection_count": 3},
        ],
        "relations": [
            {"subject_id": "obj_004", "predicate": "near", "object_id": "obj_005"},
            {"subject_id": "obj_006", "predicate": "on", "object_id": "obj_004"},
            {"subject_id": "obj_009", "predicate": "near", "object_id": "obj_011"},
            {"subject_id": "obj_013", "predicate": "near", "object_id": "obj_014"},
        ],
        "regions": [
            {"name": "入口", "object_ids": ["obj_001"],
             "center": [0.0, 0.0, 0.0]},
            {"name": "走廊", "object_ids": ["obj_002", "obj_003", "obj_008", "obj_018"],
             "center": [5.5, 3.0, 0.0]},
            {"name": "办公室", "object_ids": ["obj_004", "obj_005", "obj_006", "obj_007"],
             "center": [5.2, 3.1, 0.0]},
            {"name": "工厂车间", "object_ids": ["obj_009", "obj_010"],
             "center": [10.0, -1.5, 0.0]},
            {"name": "控制室", "object_ids": ["obj_011", "obj_012"],
             "center": [12.0, 0.25, 0.0]},
            {"name": "楼梯间", "object_ids": ["obj_013", "obj_014"],
             "center": [15.0, 2.25, 0.0]},
            {"name": "仓储区", "object_ids": ["obj_015", "obj_016"],
             "center": [18.0, -2.75, 0.0]},
            {"name": "设备间", "object_ids": ["obj_017"],
             "center": [20.0, 1.0, 0.0]},
        ],
    },
}


class SceneGraphPublisher(Node):
    def __init__(self, scenario: str = "factory", rate: float = 1.0):
        super().__init__("stub_scene_graph_publisher")
        self._pub = self.create_publisher(String, "/nav/semantic/scene_graph", 2)
        self._sg_json = json.dumps(SCENARIOS.get(scenario, SCENARIOS["factory"]),
                                   ensure_ascii=False)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Stub scene graph publisher started: scenario={scenario}, "
            f"{len(SCENARIOS.get(scenario, SCENARIOS['factory'])['objects'])} objects, "
            f"rate={rate} Hz"
        )

    def _publish(self):
        msg = String()
        msg.data = self._sg_json
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Demo scene graph publisher")
    parser.add_argument("--scenario", default="factory", choices=SCENARIOS.keys())
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = SceneGraphPublisher(scenario=args.scenario, rate=args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
