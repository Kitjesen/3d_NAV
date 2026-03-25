#!/usr/bin/env python3
"""LingTu Navigation System — dimos-style autoconnect entry point.

Usage:
    python main_nav.py                                          # stub mode
    python main_nav.py --robot thunder --dog-host 192.168.66.190  # real robot
    python main_nav.py --robot thunder --detector bpu --llm kimi  # S100P full
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from core import autoconnect


def main() -> None:
    parser = argparse.ArgumentParser(description="LingTu Navigation System")
    parser.add_argument("--robot", default="stub")
    parser.add_argument("--dog-host", default="127.0.0.1")
    parser.add_argument("--dog-port", type=int, default=13145)
    parser.add_argument("--detector", default="yoloe")
    parser.add_argument("--encoder", default="mobileclip")
    parser.add_argument("--llm", default="mock")
    parser.add_argument("--planner", default="astar")
    parser.add_argument("--tomogram", default="")
    parser.add_argument("--gateway-port", type=int, default=5050)
    parser.add_argument("--no-perception", action="store_true")
    parser.add_argument("--no-gateway", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Import modules
    from core.registry import get
    import drivers.thunder.han_dog_module  # noqa: register ThunderDriver
    try:
        import core.blueprints.stub  # noqa: register StubDogModule
    except ImportError:
        pass

    DriverCls = get("driver", args.robot)

    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    # Build with autoconnect — dimos style
    blueprints = [
        DriverCls.blueprint(dog_host=args.dog_host, dog_port=args.dog_port),
        NavigationModule.blueprint(planner=args.planner, tomogram=args.tomogram),
        SafetyRingModule.blueprint(),
    ]

    if not args.no_perception:
        try:
            from semantic.perception.semantic_perception.detector_module import DetectorModule
            from semantic.perception.semantic_perception.encoder_module import EncoderModule
            from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule
            from semantic.planner.semantic_planner.llm_module import LLMModule

            blueprints += [
                DetectorModule.blueprint(detector=args.detector),
                EncoderModule.blueprint(encoder=args.encoder),
                SemanticPlannerModule.blueprint(),
                LLMModule.blueprint(backend=args.llm),
            ]
        except ImportError:
            logging.warning("Semantic modules not available, skipping")

    if not args.no_gateway:
        try:
            from gateway.gateway_module import GatewayModule
            blueprints.append(GatewayModule.blueprint(port=args.gateway_port))
        except ImportError:
            pass
        try:
            from gateway.mcp_server import MCPServerModule
            blueprints.append(MCPServerModule.blueprint(port=8090))
        except ImportError:
            pass

    system = autoconnect(*blueprints).build()

    # Explicit safety wires (cross-name, can't auto_wire)
    # Done post-build via direct callback
    driver_name = DriverCls.__name__
    try:
        safety = system.get_module("SafetyRingModule")
        driver = system.get_module(driver_name)
        if hasattr(driver, 'stop_signal') and hasattr(safety, 'stop_cmd'):
            safety.stop_cmd._add_callback(driver.stop_signal._deliver)
    except KeyError:
        pass

    def _shutdown(signum, frame):
        print("\nShutting down...")
        system.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    system.start()

    # Print status
    n = len(system.modules)
    c = len(system.connections)
    print(f"\n{'='*50}")
    print(f"  LingTu Navigation — {n} modules, {c} connections")
    print(f"{'='*50}")
    for name, mod in system.modules.items():
        layer = f"L{mod.layer}" if mod.layer is not None else "L?"
        print(f"  [{layer}] {name}")
    print(f"{'='*50}")
    print(f"  Robot: {args.robot}  Planner: {args.planner}  LLM: {args.llm}")
    print(f"  Detector: {args.detector}  Encoder: {args.encoder}")
    print(f"\nPress Ctrl-C to stop.\n")

    try:
        if hasattr(signal, "pause"):
            signal.pause()
        else:
            import time
            while True:
                time.sleep(1)
    except (KeyboardInterrupt, EOFError):
        _shutdown(None, None)


if __name__ == "__main__":
    main()
