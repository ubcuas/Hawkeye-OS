from ultralytics import YOLOE
import cv2
import numpy as np
from rclpy.node import Node
import os

# --- Configuration ---
MODEL_PATH = "/ros2_ws/src/object_detection/yoloe-26x-seg.pt"
CONF_THRESH = 0.25
VISUALIZE = False

CLASSES = [
    "red circle", "blue circle", "green circle",
    "black circle", "yellow circle", "white circle"
]

# Lazy-loaded model singleton — initialized on first call to predict_images()
_model = None

def _get_model(node):
    """Load and cache the YOLOE model on first use."""
    global _model
    if _model is None:
        node.get_logger().info(f"Loading YOLOE model from {MODEL_PATH}...")
        _model = YOLOE(MODEL_PATH)
        _model.set_classes(CLASSES)
        node.get_logger().info("YOLOE model loaded ✅")
    return _model


def classify_color_from_mask(bgr_img, mask):
    mask_bool = mask.astype(bool)
    masked = bgr_img[mask_bool]
    if masked.size == 0:
        return "unknown", (0, 0, 0)

    mean_bgr = masked.mean(axis=0).astype(np.uint8)
    mean_hsv = cv2.cvtColor(np.uint8([[mean_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
    h, s, v = int(mean_hsv[0]), int(mean_hsv[1]), int(mean_hsv[2])

    if v < 50: return "black", tuple(int(x) for x in mean_bgr)
    if s < 30 and v > 200: return "white", tuple(int(x) for x in mean_bgr)
    if h <= 10 or h >= 160: return "red", tuple(int(x) for x in mean_bgr)
    if 20 <= h <= 35: return "yellow", tuple(int(x) for x in mean_bgr)
    if 40 <= h <= 85: return "green", tuple(int(x) for x in mean_bgr)
    if 90 <= h <= 130: return "blue", tuple(int(x) for x in mean_bgr)
    return "unknown", tuple(int(x) for x in mean_bgr)


def decode_compressed_image(msg):
    """Decode a ROS CompressedImage message into a BGR numpy array."""
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"cv2.imdecode failed for frame with encoding '{msg.format}'")
    return img


def predict_images(node, color_msg):
    model = _get_model(node)

    img = decode_compressed_image(color_msg)

    processed_images = []
    # processed images format: [((x1,y1), (x2,y2)), ...]

    # Run prediction on the decoded numpy array (single frame)
    results = model.predict(img, agnostic_nms=True, verbose=False, conf=CONF_THRESH)[0]
    output_img = img.copy()

    if results.masks is None or len(results.boxes) == 0:
        node.get_logger().info("No detections found.")
    else:
        node.get_logger().info(f"Found {len(results.boxes)} detections.")

        for i in range(len(results.boxes)):
            # 1. Extract detection data
            box = results.boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = [int(v) for v in box]
            conf = results.boxes.conf[i].cpu().item()
            cls_id = int(results.boxes.cls[i].cpu().item())
            cls_name = results.names.get(cls_id, str(cls_id))

            # 2. Extract and resize mask
            mask = results.masks.data[i].cpu().numpy()
            if mask.shape[:2] != img.shape[:2]:
                mask = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
            mask = mask > 0.5

            # 3. Classify color
            color_name, _ = classify_color_from_mask(img, mask)
            node.get_logger().info(
                f"Detection {i}: {cls_name} | {color_name} | Conf: {conf:.2f} | "
                f"BBox: ({x1}, {y1}) -> ({x2}, {y2})"
            )
            processed_images.append(((x1, y1), (x2, y2)))

            # 4. Visualization
            if VISUALIZE:
                mask_overlay = np.zeros_like(img)
                mask_overlay[mask] = (0, 255, 0)
                output_img = cv2.addWeighted(output_img, 1.0, mask_overlay, 0.4, 0)

                cv2.rectangle(output_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(output_img, label, (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if VISUALIZE:
        cv2.imshow("Result", output_img)
        node.get_logger().info("Press any key to see the next image...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return processed_images