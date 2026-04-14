#!/usr/bin/env python3
"""Minimal test: does GatewayModule's FastAPI app serve HTTP?"""
import sys, os, time, threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gateway.gateway_module import GatewayModule

gw = GatewayModule(port=5050)
gw.setup()
print(f"APP: {gw._app}", flush=True)
print(f"ROUTES: {len(gw._app.routes)}", flush=True)
for r in gw._app.routes:
    print(f"  {getattr(r, 'path', '?')} {type(r).__name__}", flush=True)

import uvicorn

config = uvicorn.Config(gw._app, host="0.0.0.0", port=5050, log_level="info")
server = uvicorn.Server(config)


def test_after_delay():
    time.sleep(5)
    import urllib.request
    for path in ["/health", "/docs", "/api/v1/health"]:
        try:
            r = urllib.request.urlopen(f"http://localhost:5050{path}", timeout=5)
            print(f"  {path}: HTTP {r.status}", flush=True)
        except Exception as e:
            print(f"  {path}: FAIL {e}", flush=True)
    print("TEST DONE", flush=True)
    os._exit(0)


threading.Thread(target=test_after_delay, daemon=True).start()
print("CALLING server.run()...", flush=True)
server.run()
print("server.run() RETURNED (unexpected)", flush=True)
