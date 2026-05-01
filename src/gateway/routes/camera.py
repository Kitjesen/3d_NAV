"""Camera snapshot routes for GatewayModule."""

from __future__ import annotations

import os
import subprocess
import tempfile

from fastapi.responses import JSONResponse
from starlette.responses import Response

from gateway.schemas import GatewayErrorResponse


def register_camera_routes(app) -> None:
    @app.get(
        "/api/v1/camera/snapshot",
        summary="Camera JPEG snapshot",
        responses={
            200: {
                "content": {
                    "image/jpeg": {"schema": {"type": "string", "format": "binary"}}
                }
            },
            500: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def camera_snapshot():
        """Grab one JPEG frame via rclpy (fastrtps, matching camera driver)."""
        out = os.path.join(tempfile.gettempdir(), "lingtu_cam_snap.jpg")
        script = os.path.join(tempfile.gettempdir(), "lingtu_cam_snap.py")
        # Write script to avoid shell escaping issues.
        with open(script, "w") as f:
            f.write(
                "import rclpy, sys\n"
                "from sensor_msgs.msg import CompressedImage\n"
                "rclpy.init()\n"
                "n=rclpy.create_node('cam_snap')\n"
                "msg=[None]\n"
                "n.create_subscription(CompressedImage,'/camera/color/image_raw/compressed',lambda m:msg.__setitem__(0,m),1)\n"
                "import time; t=time.time()\n"
                "while msg[0] is None and time.time()-t<2: rclpy.spin_once(n,timeout_sec=0.1)\n"
                "n.destroy_node(); rclpy.shutdown()\n"
                f"open('{out}','wb').write(msg[0].data) if msg[0] else None\n"
            )
        try:
            # Unset RMW to use default fastrtps (camera uses fastrtps).
            env = os.environ.copy()
            env.pop("RMW_IMPLEMENTATION", None)
            subprocess.run(
                ["bash", "-c", f"source /opt/ros/humble/setup.bash && python3 {script}"],
                capture_output=True,
                timeout=6,
                env=env,
            )
            if os.path.isfile(out) and os.path.getsize(out) > 100:
                with open(out, "rb") as f:
                    data = f.read()
                return Response(content=data, media_type="image/jpeg")
            return JSONResponse({"error": "no frame"}, status_code=503)
        except Exception as e:
            return JSONResponse({"error": str(e)}, status_code=500)
