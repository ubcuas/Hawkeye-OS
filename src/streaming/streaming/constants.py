import os

WEBRTC_SIGNALING_URL = str(os.getenv("WEBRTC_SIGNALING_URL"))
if WEBRTC_SIGNALING_URL is None or WEBRTC_SIGNALING_URL == "":
    raise ValueError("WEBRTC_SIGNALING_URL environment variable is not set")
