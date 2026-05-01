"""Traffic policy helpers for Gateway realtime channels."""

from __future__ import annotations

import asyncio
from typing import Any


DEFAULT_SSE_QUEUE_MAXSIZE = 128
DEFAULT_CLOUD_QUEUE_MAXSIZE = 2

DROP_OLDEST_POLICY = "drop_oldest"

RECOMMENDED_CLIENT_RATES_HZ: dict[str, float] = {
    "bootstrap": 0.0,
    "state": 1.0,
    "session": 1.0,
    "health": 0.2,
    "path": 2.0,
    "scene_graph": 1.0,
    "devices": 0.2,
}


def put_latest(queue: asyncio.Queue, item: Any) -> bool:
    """Put item without blocking, dropping one old item if the queue is full."""
    try:
        queue.put_nowait(item)
        return False
    except asyncio.QueueFull:
        pass

    try:
        queue.get_nowait()
    except asyncio.QueueEmpty:
        pass

    try:
        queue.put_nowait(item)
    except asyncio.QueueFull:
        return True
    return True
