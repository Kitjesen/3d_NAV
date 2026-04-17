"""Low-latency browser video via WebRTC.

WebRTCStreamModule consumes the same Image stream TeleopModule encodes
as JPEG today, but publishes it as H.264 over an RTCPeerConnection for
~100 ms glass-to-glass latency vs the 250-400 ms JPEG-over-WS path.
"""
