# DetectionData.msg
# Structured message for object detection data

string image_data          # Base64 encoded image or image bytes
string color_detection     # Detected color (e.g., "red", "blue", "green")
int32[] bounding_box       # Flattened list of coordinates [x1, y1, x2, y2, ...]
int32 confidence_level     # Confidence level (0-100)
