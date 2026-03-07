"""
ROS 2 subscriber node for image topic + YOLO segmentation.

What it does:
- Subscribes to a ROS 2 image topic
- Converts ROS Image -> OpenCV BGR frame using cv_bridge
- Runs Ultralytics YOLO segmentation
- Displays annotated output with masks

Notes:
- Use a *segmentation* model, e.g.:
    yolo11n-seg.pt
    yolo11s-seg.pt
    your_custom_model.pt
- On Jetson, CUDA must already work in PyTorch for GPU inference.
"""


import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloSegSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_seg_subscriber")

        self.declare_parameter("topic_name", "/camera/color/image_raw")
        self.declare_parameter("model_path", "yoloe-26x-seg.pt")
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("use_cuda", True)

        self.topic_name = self.get_parameter("topic_name").value
        self.model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.use_cuda = bool(self.get_parameter("use_cuda").value)

        self.bridge = CvBridge()

        # Pick device
        if self.use_cuda and torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"

        self.get_logger().info(f"Loading model: {self.model_path}")
        self.get_logger().info(f"Running YOLO on device: {self.device}")

        self.model = YOLO(self.model_path)
        self.model.set_classes(["red circle", "blue circle", "green circle", "black circle", "yellow circle" ,"white circle"])


        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.processing = False
        self.frame_count = 0

        self.get_logger().info(f"Subscribed to: {self.topic_name}")


    def image_callback(self, msg: Image):
        # Drop frame if previous inference still running
        if self.processing:
            return

        self.processing = True

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Run YOLO segmentation
            # results = self.model.predict(
            #     source=frame_bgr,
            #     conf=self.conf,
            #     imgsz=self.imgsz,
            #     device=self.device,
            #     verbose=False,
            # )
            results = self.model.predict(frame_bgr, agnostic_nms=True, verbose=False)[0]

            annotated = frame_bgr
            if results and len(results) > 0:
                # result.plot() returns annotated BGR image
                annotated = results[0].plot()

                # Optional logging
                num_boxes = 0 if results[0].boxes is None else len(results[0].boxes)
                num_masks = 0
                if results[0].masks is not None and results[0].masks.data is not None:
                    num_masks = int(results[0].masks.data.shape[0])

                self.frame_count += 1
                if self.frame_count % 5 == 0:
                    self.get_logger().info(
                        f"Frame {self.frame_count}: detections={num_boxes}, masks={num_masks}"
                    )

            cv2.imshow("YOLO Segmentation", annotated)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")

        finally:
            self.processing = False

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()(yolo_env) uas-jc@uasjc-desktop:~/yolo_2026_experiment$ 

