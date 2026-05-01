"""WebSocket route registration for GatewayModule."""

from __future__ import annotations

import asyncio
import json
import logging

from core.msgs.geometry import Twist

logger = logging.getLogger(__name__)


def register_realtime_routes(app, gw) -> None:
    from starlette.websockets import WebSocket as StarletteWebSocket
    from starlette.websockets import WebSocketDisconnect as StarletteWebSocketDisconnect

    async def ws_teleop_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        gw._teleop_clients += 1
        tm = gw._teleop_module
        if tm is not None:
            tm.on_client_connect()
        logger.info("Teleop WS connected (%d clients)", gw._teleop_clients)

        async def send_frames():
            while True:
                with gw._jpeg_lock:
                    frame = gw._latest_jpeg
                if frame:
                    try:
                        await ws.send_bytes(frame)
                    except Exception as e:
                        logger.debug("teleop frame send failed: %s", e)
                        break
                await asyncio.sleep(0.1)

        frame_task = asyncio.create_task(send_frames())
        try:
            while True:
                msg = await ws.receive()
                if msg["type"] == "websocket.disconnect":
                    break
                raw = msg.get("text") or msg.get("bytes", b"").decode()
                if not raw:
                    continue
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                msg_type = data.get("type", "")
                if msg_type == "joy":
                    gw._teleop_on_joy(
                        float(data.get("lx", 0)),
                        float(data.get("ly", 0)),
                        float(data.get("az", 0)),
                    )
                elif msg_type == "stop":
                    gw.stop_cmd.publish(2)
                    if tm is not None:
                        tm.force_release()
                    else:
                        gw.cmd_vel.publish(Twist())
        except StarletteWebSocketDisconnect:
            pass
        finally:
            frame_task.cancel()
            gw._teleop_clients = max(0, gw._teleop_clients - 1)
            if tm is not None:
                tm.on_client_disconnect()
            elif gw._teleop_clients == 0:
                gw._teleop_release()
            logger.info("Teleop WS disconnected (%d clients)", gw._teleop_clients)

    async def ws_cloud_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        q, latest = gw._cloud_subscribe()
        try:
            if latest is not None:
                await ws.send_bytes(latest)
            while True:
                buf = await q.get()
                await ws.send_bytes(buf)
        except StarletteWebSocketDisconnect:
            pass
        except Exception as e:
            logger.debug("cloud ws send failed: %s", e)
        finally:
            gw._cloud_unsubscribe(q)

    app.add_websocket_route("/ws/teleop", ws_teleop_endpoint)
    app.add_websocket_route("/ws/cloud", ws_cloud_endpoint)
