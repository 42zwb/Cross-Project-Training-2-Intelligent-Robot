"""Run CPU-friendly YOLOv8 object detection on a ROS 2 image stream."""

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloDetector(Node):
    """Subscribe to RGB images and publish 2D detections and an overlay."""

    def __init__(self):
        super().__init__('yolov8_detector')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('image_size', 320)
        self.declare_parameter('device', 'cpu')

        image_topic = self.get_parameter('image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.device = self.get_parameter('device').value
        if not isinstance(image_topic, str) or not image_topic.strip():
            raise ValueError('image_topic must be a non-empty topic name')
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError('confidence must be between 0 and 1')
        if self.image_size < 64:
            raise ValueError('image_size must be at least 64 pixels')
        if not isinstance(self.model_path, str) or not self.model_path.strip():
            raise ValueError('model_path must be a non-empty path or model name')
        if not isinstance(self.device, str) or not self.device.strip():
            raise ValueError('device must be "cpu" or a supported PyTorch device')

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'Ultralytics is missing. Activate the project YOLO virtualenv '
                'before starting this node.'
            ) from exc

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)
        self.detections_publisher = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10)
        self.image_publisher = self.create_publisher(
            Image, '/yolo/annotated', 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.last_error_log = 0.0
        self.get_logger().info(
            f'YOLO ready: model={self.model_path}, device={self.device}, '
            f'input={self.image_size}, image_topic={image_topic}')

    def on_image(self, message):
        """Infer one frame and publish boxes plus an annotated image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            result = self.model.predict(
                frame,
                conf=self.confidence,
                imgsz=self.image_size,
                device=self.device,
                max_det=100,
                verbose=False,
            )[0]
            detection_array = Detection2DArray()
            detection_array.header = message.header
            names = result.names
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                coordinates = boxes.xyxy.cpu().tolist()
                class_ids = boxes.cls.cpu().tolist()
                scores = boxes.conf.cpu().tolist()
                for (x_min, y_min, x_max, y_max), class_id, score in zip(
                        coordinates, class_ids, scores):
                    class_id = int(class_id)
                    label = names.get(class_id, str(class_id)) if isinstance(
                        names, dict) else names[class_id]
                    detection = Detection2D()
                    detection.header = message.header
                    detection.bbox = BoundingBox2D()
                    detection.bbox.center.position.x = (x_min + x_max) / 2.0
                    detection.bbox.center.position.y = (y_min + y_max) / 2.0
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = max(0.0, x_max - x_min)
                    detection.bbox.size_y = max(0.0, y_max - y_min)
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(label)
                    hypothesis.hypothesis.score = float(score)
                    detection.results.append(hypothesis)
                    detection_array.detections.append(detection)

            self.detections_publisher.publish(detection_array)
            annotated = result.plot()
            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8')
            annotated_message.header = message.header
            self.image_publisher.publish(annotated_message)
            labels = [
                detection.results[0].hypothesis.class_id
                for detection in detection_array.detections
            ]
            self.get_logger().info(
                f'detections={len(labels)} labels={labels}',
                throttle_duration_sec=2.0,
            )
        except Exception as exc:  # Keep a bad frame from terminating the node.
            now = time.monotonic()
            if now - self.last_error_log >= 5.0:
                self.get_logger().error(f'Image inference failed: {exc}')
                self.last_error_log = now


def main(args=None):
    """Start the YOLO node."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as exc:
        print(f'yolov8_detector: {exc}')
        exit_code = 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
