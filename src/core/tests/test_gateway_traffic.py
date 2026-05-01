from __future__ import annotations

import pytest


pytest.importorskip("fastapi")


def test_sse_slow_client_keeps_latest_events_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue = gateway._sse_subscribe()

    for seq in range(gateway._sse_queue_maxsize + 3):
        gateway.push_event({"type": "tick", "seq": seq})

    assert queue.qsize() == gateway._sse_queue_maxsize
    assert queue.get_nowait()["seq"] == 3

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["clients"] == 1
    assert stats["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert stats["sse"]["published_events"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["dropped_events"] == 3
    assert stats["sse"]["drop_policy"] == DROP_OLDEST_POLICY


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
