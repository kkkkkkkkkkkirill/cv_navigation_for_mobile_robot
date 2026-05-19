#!/usr/bin/env python3
"""
YOLOv8 object detector node.

Subscribes:
  /camera/image           sensor_msgs/Image   RGB stream from D435

Publishes:
  /yolo/debug_image       sensor_msgs/Image   annotated frame with boxes + labels

Uses ultralytics YOLOv8 on CPU. Default model is yolov8n.pt (smallest, ~6MB)
which is auto-downloaded on first use to ~/.cache/ultralytics.
"""
from __future__ import annotations

import os
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_node')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('debug_topic', '/yolo/debug_image')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('downscale', 1.0)

        image_topic = str(self.get_parameter('image_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        model_path = str(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('confidence_threshold').value)
        self.iou = float(self.get_parameter('iou_threshold').value)
        self.downscale = float(self.get_parameter('downscale').value)

        # lazy-import ultralytics so import errors are caught with a friendly msg
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().error(
                f'ultralytics is not installed: {exc}. pip install ultralytics'
            )
            raise

        self.get_logger().info(f'loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        # warm up
        _ = self.model.predict(np.zeros((320, 320, 3), dtype=np.uint8),
                               verbose=False, conf=self.conf, iou=self.iou)
        self.class_names = self.model.names

        self.create_subscription(Image, image_topic, self.image_cb, 5)
        self.debug_pub = self.create_publisher(Image, debug_topic, 5)
        self.detections_pub = self.create_publisher(String, '/yolo/detections', 5)
        self._frame_count = 0
        self._first_logged = False

        self.get_logger().info(
            f'yolo_node ready: image={image_topic} debug={debug_topic} '
            f'conf={self.conf} iou={self.iou} downscale={self.downscale}'
        )

    @staticmethod
    def _decode_image(msg: Image) -> Optional[np.ndarray]:
        enc = msg.encoding
        if enc == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        if enc == 'mono8':
            mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        return None

    @staticmethod
    def _encode_image(img_bgr: np.ndarray, header) -> Image:
        out = Image()
        out.header = header
        out.height = img_bgr.shape[0]
        out.width = img_bgr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = img_bgr.shape[1] * 3
        out.data = img_bgr.tobytes()
        return out

    def image_cb(self, msg: Image) -> None:
        frame = self._decode_image(msg)
        if frame is None:
            return

        if not self._first_logged:
            self.get_logger().info(
                f'first frame: {msg.width}x{msg.height} ({msg.encoding})'
            )
            self._first_logged = True

        inp = frame
        if self.downscale != 1.0 and self.downscale > 0:
            h, w = frame.shape[:2]
            inp = cv2.resize(frame, (int(w * self.downscale), int(h * self.downscale)))

        results = self.model.predict(
            inp, verbose=False, conf=self.conf, iou=self.iou, imgsz=max(inp.shape[:2])
        )

        annotated = frame.copy()
        scale_back_x = frame.shape[1] / inp.shape[1]
        scale_back_y = frame.shape[0] / inp.shape[0]

        num = 0
        detected_classes: list[str] = []
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                num += 1
                xyxy = b.xyxy[0].cpu().numpy().astype(float)
                x1 = int(xyxy[0] * scale_back_x)
                y1 = int(xyxy[1] * scale_back_y)
                x2 = int(xyxy[2] * scale_back_x)
                y2 = int(xyxy[3] * scale_back_y)
                cls_id = int(b.cls[0].item()) if b.cls is not None else -1
                conf = float(b.conf[0].item()) if b.conf is not None else 0.0
                label = self.class_names.get(cls_id, f'cls{cls_id}') if isinstance(self.class_names, dict) else str(cls_id)
                detected_classes.append(label)
                color = self._color_for_class(cls_id)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                txt = f'{label} {conf:.2f}'
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
                cv2.putText(annotated, txt, (x1 + 3, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        banner_color = (0, 200, 0) if num > 0 else (160, 160, 160)
        cv2.putText(annotated, f'YOLOv8 detections: {num}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, banner_color, 2)

        self.debug_pub.publish(self._encode_image(annotated, msg.header))
        det_msg = String()
        det_msg.data = ','.join(detected_classes)
        self.detections_pub.publish(det_msg)
        self._frame_count += 1

    @staticmethod
    def _color_for_class(cls_id: int):
        # deterministic distinct colors
        np.random.seed(cls_id * 9973 + 7)
        c = np.random.randint(64, 255, size=3).tolist()
        return (int(c[0]), int(c[1]), int(c[2]))


def main() -> None:
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
