import cv2
import numpy as np
import torch
from rclpy.node import Node
import os

# --- PyTorch 2.6 compatibility ---
# PyTorch 2.6+ defaults torch.load(weights_only=True) which blocks loading
# YOLOE checkpoints containing custom classes (YOLOESegModel, etc.).
# Ultralytics calls torch.load internally in YOLOE(), set_classes(), and
# predict(), so we must patch the default globally.
# _original_torch_load = torch.load
# def _patched_torch_load(*args, **kwargs):
#     kwargs.setdefault("weights_only", False)
#     return _original_torch_load(*args, **kwargs)
# torch.load = _patched_torch_load

from ultralytics import YOLOE

# --- Configuration ---
MODEL_PATH = "/ros2_ws/src/object_detection/yoloe-26x-seg.pt"
CONF_THRESH = 0.25
IMGSZ = 960
USE_CUDA = True
VISUALIZE = False

class YoloDetector:
    def __init__(self, node, model_path="/ros2_ws/src/object_detection/yoloe-26x-seg.pt", 
                 conf_thresh=0.25, imgsz=IMGSZ, use_cuda=True, visualize=VISUALIZE, logger=None):
        self.node = node
        self.model_path = model_path
        self.conf_thresh = conf_thresh
        self.imgsz = imgsz
        self.visualize = visualize
        self.logger = logger
        
        self.classes = [
            "red circle", "blue circle", "green circle",
            "black circle", "yellow circle", "white circle"
        ]

        if use_cuda and torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"
            
        self.node.get_logger().info(f"Loading YOLOE model from {self.model_path} on {self.device}...")
        self.model = YOLOE(self.model_path)
        self.model.set_classes(self.classes)
        self.node.get_logger().info("YOLOE model loaded ✅")


    def classify_color_from_mask(self, bgr_img, mask):
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


    def decode_compressed_image(self, msg):
        """Decode a ROS CompressedImage message into a BGR numpy array."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"cv2.imdecode failed for frame with encoding '{msg.format}'")
        return img


    def predict(self, color_msg):

        img = self.decode_compressed_image(color_msg)

        processed_images = []
        # processed images format: [((x1,y1), (x2,y2)), ...]

        # Run prediction on the decoded numpy array (single frame)
        self.node.get_logger().info(self.device)
        results = self.model.predict(img, agnostic_nms=True, verbose=False, conf=CONF_THRESH, imgsz=IMGSZ, device=self.device)[0]
        output_img = img.copy()

        if results.masks is None or len(results.boxes) == 0:
            self.node.get_logger().info("No detections found.")
            return processed_images
        
        # batch
        boxes_xyxy = results.boxes.xyxy.cpu().numpy()   # shape: (N, 4)
        confs      = results.boxes.conf.cpu().numpy()   # shape: (N,)
        cls_ids    = results.boxes.cls.cpu().numpy().astype(int)  # shape: (N,)
        masks_data = results.masks.data.cpu().numpy()   # shape: (N, H, W)

        self.node.get_logger().info(f"Found {len(results.boxes)} detections.")

        for i in range(len(results.boxes)):
            # 1. Extract detection data
            x1, y1, x2, y2 = boxes_xyxy[i].astype(int)
            conf     = confs[i]
            cls_id   = cls_ids[i]
            cls_name = results.names.get(cls_id, str(cls_id))

            mask = masks_data[i]
            if mask.shape[:2] != img.shape[:2]:
                mask = cv2.resize(mask, (img.shape[1], img.shape[0]),
                                  interpolation=cv2.INTER_LINEAR)
            mask = mask > 0.5

            # 3. Classify color
            color_name, _ = self.classify_color_from_mask(img, mask)
            self.node.get_logger().info(
                f"Detection {i}: {cls_name} | {color_name} | Conf: {conf:.2f} | "
                f"BBox: ({x1}, {y1}) -> ({x2}, {y2})"
            )
            #temp
            conf = 0.6
            
            processed_images.append((
                (x1, y1, x2, y2), 
                conf, 
                color_name)
            )

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
            self.node.get_logger().info("Press any key to see the next image...")
            cv2.waitKey(1)
            #cv2.destroyAllWindows()

        return processed_images