"""Autonomous exploration backends.

Provides frontier-based (``WavefrontFrontierExplorer`` in ``nav/``) and
hierarchical TARE-based exploration wrapped as LingTu Modules. The TARE
backend launches the CMU ROS2 node as a NativeModule subprocess and bridges
its DDS topics into the framework ports.
"""
