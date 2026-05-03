from __future__ import annotations

import asyncio
import threading

import pytest


pytest.importorskip("fastapi")


def test_sse_slow_client_keeps_latest_events_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY, SSE_EVENT_SCHEMA_VERSION

    gateway = GatewayModule()
    queue = gateway._sse_subscribe()

    for seq in range(gateway._sse_queue_maxsize + 3):
        gateway.push_event({"type": "tick", "seq": seq})

    assert queue.qsize() == gateway._sse_queue_maxsize
    retained = queue.get_nowait()
    assert retained["seq"] == 3
    assert retained["event_id"] == 4
    assert retained["schema_version"] == SSE_EVENT_SCHEMA_VERSION
    assert retained["ts"] > 0

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["clients"] == 1
    assert stats["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert stats["sse"]["latest_event_id"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["published_events"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["dropped_events"] == 3
    assert stats["sse"]["drop_policy"] == DROP_OLDEST_POLICY


def test_sse_raster_events_are_client_bound_and_throttled():
    import numpy as np

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._sse_raster_min_interval_s = 60.0
    grid = np.full((2, 3), 42, dtype=np.uint8)

    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})
    assert gateway._traffic_stats_snapshot()["sse"]["published_events"] == 0

    queue = gateway._sse_subscribe()
    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})

    event = queue.get_nowait()
    assert event["type"] == "costmap"
    assert event["rows"] == 2
    assert event["cols"] == 3
    assert event["grid_b64"]

    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})
    assert queue.empty()

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["suppressed_events"]["costmap"] == 1
    assert stats["sse"]["raster_min_interval_s"] == 60.0


def test_slope_grid_defaults_to_metadata_and_can_enable_inline_payload():
    import numpy as np

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._sse_raster_min_interval_s = 0.0
    queue = gateway._sse_subscribe()

    gateway._on_slope_grid({"grid": np.ones((2, 2)), "resolution": 0.2, "origin": [0.0, 0.0]})
    event = queue.get_nowait()

    assert event["type"] == "slope_grid"
    assert event["payload"] == "omitted"
    assert event["available"] is True
    assert event["rows"] == 2
    assert event["cols"] == 2
    assert "grid_b64" not in event

    gateway._sse_slope_payload_enabled = True
    gateway._on_slope_grid({"grid": np.ones((2, 2)), "resolution": 0.2, "origin": [0.0, 0.0]})
    event = queue.get_nowait()

    assert event["type"] == "slope_grid"
    assert event["payload"] == "inline"
    assert event["grid_b64"]


def test_cloud_slow_client_keeps_latest_frames_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue, latest = gateway._cloud_subscribe()

    assert latest is None

    for seq in range(gateway._cloud_queue_maxsize + 2):
        gateway._publish_cloud_frame(bytes([seq]))

    assert queue.qsize() == gateway._cloud_queue_maxsize
    assert queue.get_nowait() == bytes([2])

    stats = gateway._traffic_stats_snapshot()
    assert stats["cloud"]["clients"] == 1
    assert stats["cloud"]["queue_maxsize"] == gateway._cloud_queue_maxsize
    assert stats["cloud"]["published_frames"] == gateway._cloud_queue_maxsize + 2
    assert stats["cloud"]["dropped_frames"] == 2
    assert stats["cloud"]["drop_policy"] == DROP_OLDEST_POLICY
    assert stats["cloud"]["latest_seq"] == gateway._cloud_queue_maxsize + 2


def test_gateway_health_and_bootstrap_expose_traffic_policy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()
    gateway._sse_subscribe()
    gateway._cloud_subscribe()

    health = gateway.health()
    bootstrap = build_app_bootstrap(gateway)

    assert health["gateway"]["traffic"]["sse"]["clients"] == 1
    assert health["gateway"]["traffic"]["cloud"]["clients"] == 1
    assert bootstrap["traffic"]["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert bootstrap["traffic"]["cloud"]["queue_maxsize"] == gateway._cloud_queue_maxsize
    assert bootstrap["traffic"]["recommended_client_rates_hz"]["state"] == 1.0
    assert bootstrap["traffic"]["client_policy"]["large_event_policy"]["slope_grid_payload"] == "metadata_sse"


def test_sse_message_format_keeps_eventsource_onmessage_contract():
    from gateway.services.traffic import (
        SSE_EVENT_SCHEMA_VERSION,
        format_sse_message,
        normalize_sse_event,
    )

    event = normalize_sse_event({"type": "mission_status", "data": {"state": "IDLE"}}, event_id=7, now=123.0)
    text = format_sse_message(event, retry_ms=3000)

    assert text.startswith("retry: 3000\nid: 7\ndata: ")
    assert "\nevent:" not in text
    assert '"schema_version":1' in text
    assert '"event_id":7' in text
    assert '"ts":123.0' in text
    assert event["schema_version"] == SSE_EVENT_SCHEMA_VERSION


def test_sse_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue = gateway._sse_subscribe()

        thread = threading.Thread(
            target=lambda: gateway.push_event({"type": "threaded", "data": 1})
        )
        thread.start()
        try:
            event = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._sse_unsubscribe(queue)

        assert event["type"] == "threaded"
        assert event["event_id"] == 1
        assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0

    asyncio.run(_run())


def test_cloud_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue, latest = gateway._cloud_subscribe()
        assert latest is None

        thread = threading.Thread(
            target=lambda: gateway._publish_cloud_frame(b"PCL-threaded")
        )
        thread.start()
        try:
            frame = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._cloud_unsubscribe(queue)

        assert frame == b"PCL-threaded"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0

    asyncio.run(_run())


def test_camera_websocket_is_camera_only_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.camera_clients = 0
            self.teleop_clients = 0

        def on_camera_client_connect(self):
            self.camera_clients += 1

        def on_camera_client_disconnect(self):
            self.camera_clients -= 1

        def on_client_connect(self):
            self.teleop_clients += 1

        def on_client_disconnect(self):
            self.teleop_clients -= 1

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    gateway.push_jpeg(b"\xff\xd8\xffcamera")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/camera") as ws:
        assert ws.receive_bytes() == b"\xff\xd8\xffcamera"
        assert tracker.camera_clients == 1
        assert tracker.teleop_clients == 0
        assert gateway._teleop_clients == 0

    assert tracker.camera_clients == 0
    assert tracker.teleop_clients == 0


def test_cloud_websocket_sends_latest_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._publish_cloud_frame(b"PCL0")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/cloud") as ws:
        assert ws.receive_bytes() == b"PCL0"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 1
        gateway._publish_cloud_frame(b"PCL1")
        assert ws.receive_bytes() == b"PCL1"

    assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0


def test_teleop_websocket_ignores_malformed_frames_without_motion():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.clients = 0

        def on_client_connect(self):
            self.clients += 1

        def on_client_disconnect(self):
            self.clients -= 1

        def force_release(self):
            pass

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    sent_cmds = []
    gateway.cmd_vel._add_callback(sent_cmds.append)

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop") as ws:
        ws.send_bytes(b"\xff")
        ws.send_text("[1, 2, 3]")
        ws.send_text("not json")
        ws.send_text('{"type":"joy","lx":"bad","ly":0,"az":0}')
        ws.send_text('{"type":"unknown"}')
        assert gateway._teleop_clients == 1
        assert tracker.clients == 1
        assert sent_cmds == []

    assert gateway._teleop_clients == 0
    assert tracker.clients == 0
    assert sent_cmds == []
