#!/usr/bin/env python3
"""
ArUco detector + solvePnP pose estimation node for Gazebo simulation.

Subscribes to:
  /camera/image           sensor_msgs/Image     RGB stream from D435
  /camera/camera_info     sensor_msgs/CameraInfo intrinsics from Gazebo

Publishes:
  /aruco/debug_image      sensor_msgs/Image     annotated image for RViz

Annotations on debug image:
  - green polygon around each detected marker
  - id label
  - euclidean distance from camera to marker (||tvec||)
  - horizontal offset (left/right)
  - vertical offset (above/below)
  - forward distance (tvec[2], depth toward camera optical axis)
  - X(red) / Y(green) / Z(blue) marker frame axes drawn through projectPoints

Marker side defaults to 0.30 m (matches Gazebo world ArUco models).
Camera intrinsics are read from /camera/camera_info, NOT hardcoded.
"""
import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


DEFAULT_MARKER_SIDE_M = 0.30


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.declare_parameter('marker_side_m', DEFAULT_MARKER_SIDE_M)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('debug_topic', '/aruco/debug_image')
        self.declare_parameter('dictionary', 'DICT_6X6_250')

        self.marker_side = float(self.get_parameter('marker_side_m').value)
        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        dict_name = str(self.get_parameter('dictionary').value)

        try:
            dict_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().warn(f'unknown dictionary "{dict_name}", falling back to DICT_6X6_250')
            dict_id = cv2.aruco.DICT_6X6_250

        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        # 3D corners of the marker in its own frame, centered at marker origin.
        # Order required by SOLVEPNP_IPPE_SQUARE: TL, TR, BR, BL with Y-up.
        s = self.marker_side / 2.0
        self.object_corners_3d = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

        self.camera_matrix = None
        self.dist_coeffs = None
        self._info_logged = False
        self._first_frame_logged = False

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

        self.get_logger().info(
            f'aruco_node ready: side={self.marker_side:.2f}m dict={dict_name} '
            f'image={image_topic} info={camera_info_topic} debug={debug_topic}'
        )

    # ---------- camera_info ----------
    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5, dtype=np.float64)
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            return
        self.camera_matrix = K
        self.dist_coeffs = D
        self.get_logger().info(
            f'camera intrinsics: fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
            f'cx={K[0,2]:.1f} cy={K[1,2]:.1f} dist={list(np.round(D, 4))}'
        )

    # ---------- image (de)coding without cv_bridge ----------
    @staticmethod
    def _decode_image(msg: Image):
        enc = msg.encoding
        if enc == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        elif enc == 'mono8':
            mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            img = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        else:
            return None
        return img

    @staticmethod
    def _encode_image(img_bgr, header):
        out = Image()
        out.header = header
        out.height = img_bgr.shape[0]
        out.width = img_bgr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = img_bgr.shape[1] * 3
        out.data = img_bgr.tobytes()
        return out

    # ---------- main image callback ----------
    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            if not self._info_logged:
                self.get_logger().warn('waiting for camera_info...', throttle_duration_sec=5.0)
                self._info_logged = True
            return

        frame = self._decode_image(msg)
        if frame is None:
            self.get_logger().warn(f'unsupported image encoding: {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return

        if not self._first_frame_logged:
            self.get_logger().info(f'first frame received: {msg.width}x{msg.height} ({msg.encoding})')
            self._first_frame_logged = True

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        count = 0
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                pts = corners[i].reshape(4, 2)

                # 1. green polygon around marker
                pts_int = pts.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(frame, [pts_int], True, (0, 255, 0), 2)

                # 2. solvePnP for square coplanar markers
                image_points = pts.astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_corners_3d,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue

                count += 1
                tx, ty, tz = tvec.flatten()
                distance = float(np.linalg.norm(tvec))

                # In camera optical frame:
                #   tx > 0 => marker is to the right of optical axis
                #   ty > 0 => marker is below optical axis
                #   tz    => forward distance along optical axis
                if abs(tx) < 0.02:
                    h_text = 'centered horizontally'
                elif tx > 0:
                    h_text = f'{abs(tx):.2f}m right'
                else:
                    h_text = f'{abs(tx):.2f}m left'

                if abs(ty) < 0.02:
                    v_text = 'centered vertically'
                elif ty > 0:
                    v_text = f'{abs(ty):.2f}m below'
                else:
                    v_text = f'{abs(ty):.2f}m above'

                cx_px = int(pts[:, 0].mean())
                cy_px = int(pts[:, 1].mean())

                # 3. draw axes manually (drawFrameAxes prints noisy warnings on some OpenCV builds)
                self._draw_axes(frame, rvec, tvec, self.marker_side * 0.5)

                # 4. text annotations near marker center
                cv2.putText(frame, f'id:{int(marker_id)}  d={distance:.2f}m',
                            (cx_px - 80, cy_px - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, h_text,
                            (cx_px - 80, cy_px + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
                cv2.putText(frame, v_text,
                            (cx_px - 80, cy_px + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
                cv2.putText(frame, f'fwd={tz:.2f}m',
                            (cx_px - 80, cy_px + 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

        # status banner top-left
        color = (0, 255, 0) if count > 0 else (160, 160, 160)
        cv2.putText(frame, f'markers detected: {count}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        out = self._encode_image(frame, msg.header)
        self.debug_pub.publish(out)

    def _draw_axes(self, frame, rvec, tvec, length):
        axis_pts = np.array([
            [0.0, 0.0, 0.0],
            [length, 0.0, 0.0],
            [0.0, length, 0.0],
            [0.0, 0.0, length],
        ], dtype=np.float64)
        proj, _ = cv2.projectPoints(
            axis_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        proj = proj.reshape(-1, 2).astype(int)
        origin = tuple(proj[0].tolist())
        cv2.line(frame, origin, tuple(proj[1].tolist()), (0, 0, 255), 2)  # X red
        cv2.line(frame, origin, tuple(proj[2].tolist()), (0, 255, 0), 2)  # Y green
        cv2.line(frame, origin, tuple(proj[3].tolist()), (255, 0, 0), 2)  # Z blue


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
