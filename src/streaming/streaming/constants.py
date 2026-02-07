import os

WEBRTC_SIGNALING_URL = os.getenv("WEBRTC_SIGNALING_URL")
if WEBRTC_SIGNALING_URL is None:
    raise ValueError("WEBRTC_SIGNALING_URL environment variable is not set")
