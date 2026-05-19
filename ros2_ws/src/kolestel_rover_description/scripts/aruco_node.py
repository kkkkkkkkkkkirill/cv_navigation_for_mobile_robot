#!/usr/bin/env python3
"""
ArUco detector + solvePnP pose estimation + planar pose fuser.

This node is the single source of truth for the robot's map-frame pose. It
maintains an internal planar state (x, y, yaw) in `map`:

    Predict (on each /odom message at ~50 Hz):
        delta_body = inv(T_OB_prev) * T_OB_now
        T_WB_filt  = T_WB_filt * delta_body

    Update (on each accepted ArUco detection, ~30 Hz):
        T_WB_filt  = lerp(T_WB_filt, T_WB_meas, alpha)   # planar lerp

The TF `map -> odom` is *derived* from the filter as
    T_map_odom = T_WB_filt * inv(T_OB_now)
so the existing TF tree still works for downstream consumers (RViz, the web
UI bridge), while the filter state remains smooth across measurements.

A /aruco/pose (PoseStamped) topic exposes the fused pose directly so that
the controller can consume the same pose without a TF lookup race.

Marker frame convention at yaw=0 (matches the south-readable face):
    marker_X = world +X
    marker_Y = world +Z
    marker_Z = world -Y     (out of plate, toward an observer south of marker)
Verified analytically against a synthetic test case.

Detection / annotation topics are unchanged:
    /camera/image           sensor_msgs/Image
    /camera/camera_info     sensor_msgs/CameraInfo
    /aruco/debug_image      sensor_msgs/Image
"""
from __future__ import annotations

import math
import os
from threading import Lock
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


DEFAULT_MARKER_SIDE_M = 0.30


# ---------- SE(3) / rotation helpers (no external deps) ----------


def quat_to_mat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (qy * qy + qz * qz),  s * (qx * qy - qz * qw),      s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw),      1 - s * (qx * qx + qz * qz),  s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw),      s * (qy * qz + qx * qw),      1 - s * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def mat_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return (qx, qy, qz, qw)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def inv_se3(T: np.ndarray) -> np.ndarray:
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    out = np.eye(4)
    out[0:3, 0:3] = R.T
    out[0:3, 3] = -R.T @ t
    return out


def transform_msg_to_mat(ts: TransformStamped) -> np.ndarray:
    T = np.eye(4)
    q = ts.transform.rotation
    T[0:3, 0:3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[0, 3] = ts.transform.translation.x
    T[1, 3] = ts.transform.translation.y
    T[2, 3] = ts.transform.translation.z
    return T


def odom_pose_to_mat(odom: Odometry) -> np.ndarray:
    T = np.eye(4)
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    T[0:3, 0:3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z
    return T


def planar_mat(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[0:3, 0:3] = [
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ]
    T[0, 3] = x
    T[1, 3] = y
    return T


def mat_to_planar(T: np.ndarray) -> Tuple[float, float, float]:
    x = float(T[0, 3])
    y = float(T[1, 3])
    yaw = float(math.atan2(T[1, 0], T[0, 0]))
    return x, y, yaw


def make_marker_world_mat(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    """T_WM: marker frame in world frame.

    yaw=0 => readable face on -Y (robot driving NORTH sees it head-on).
    Verified analytically against a synthetic test case (see project notes).
    """
    R0 = np.array([
        [ 1.0, 0.0,  0.0],
        [ 0.0, 0.0, -1.0],
        [ 0.0, 1.0,  0.0],
    ], dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    T = np.eye(4)
    T[0:3, 0:3] = Rz @ R0
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def shortest_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ---------- node ----------


class ArucoNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_node')

        # ---- detection / topic params ----
        self.declare_parameter('marker_side_m', DEFAULT_MARKER_SIDE_M)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('debug_topic', '/aruco/debug_image')
        self.declare_parameter('dictionary', 'DICT_6X6_250')

        # ---- frames ----
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')

        # ---- pose fuser ----
        self.declare_parameter('enable_drift_correction', True)
        self.declare_parameter('marker_world_poses_path', '')
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('filter_alpha_xy', 0.10)     # per-detection blend in xy
        self.declare_parameter('filter_alpha_yaw', 0.10)    # per-detection blend in yaw
        self.declare_parameter('min_marker_distance_m', 0.8)
        self.declare_parameter('max_marker_distance_m', 2.5)
        self.declare_parameter('max_marker_obliqueness_deg', 45.0)
        self.declare_parameter('measurement_outlier_xy_m', 0.7)
        self.declare_parameter('measurement_outlier_yaw_rad', 0.5)
        self.declare_parameter('consecutive_outliers_for_resync', 5)
        self.declare_parameter('tf_publish_rate_hz', 50.0)

        self.marker_side = float(self.get_parameter('marker_side_m').value)
        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        dict_name = str(self.get_parameter('dictionary').value)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_optical_frame = str(self.get_parameter('camera_optical_frame').value)

        self.enable_drift_correction = bool(self.get_parameter('enable_drift_correction').value)
        marker_world_poses_path = str(self.get_parameter('marker_world_poses_path').value)
        self.alpha_xy = float(self.get_parameter('filter_alpha_xy').value)
        self.alpha_yaw = float(self.get_parameter('filter_alpha_yaw').value)
        self.min_marker_distance_m = float(self.get_parameter('min_marker_distance_m').value)
        self.max_marker_distance_m = float(self.get_parameter('max_marker_distance_m').value)
        self.max_marker_obliqueness_rad = math.radians(
            float(self.get_parameter('max_marker_obliqueness_deg').value)
        )
        self.outlier_xy_m = float(self.get_parameter('measurement_outlier_xy_m').value)
        self.outlier_yaw_rad = float(self.get_parameter('measurement_outlier_yaw_rad').value)
        self.consec_outliers_for_resync = int(self.get_parameter('consecutive_outliers_for_resync').value)
        tf_publish_rate = float(self.get_parameter('tf_publish_rate_hz').value)

        # ---- ArUco detector ----
        try:
            dict_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().warn(f'unknown dictionary "{dict_name}", falling back to DICT_6X6_250')
            dict_id = cv2.aruco.DICT_6X6_250

        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        # solvePnP object points (TL, TR, BR, BL) — SOLVEPNP_IPPE_SQUARE ordering.
        s = self.marker_side / 2.0
        self.object_corners_3d = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

        # ---- runtime state ----
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._info_logged = False
        self._first_frame_logged = False

        self.marker_world_poses: Dict[int, Dict[str, float]] = {}
        if self.enable_drift_correction:
            self.marker_world_poses = self._load_marker_world_poses(marker_world_poses_path)

        self._lock = Lock()
        self._latest_odom: Optional[Odometry] = None
        self._prev_odom_mat: Optional[np.ndarray] = None
        self._base_to_camera_mat: Optional[np.ndarray] = None
        self._base_to_camera_warned = False

        # Filter state: planar pose T_WB_filtered. Initialize to depot.
        init_x = float(self.get_parameter('initial_pose_x').value)
        init_y = float(self.get_parameter('initial_pose_y').value)
        init_yaw = float(self.get_parameter('initial_pose_yaw').value)
        self._T_WB = planar_mat(init_x, init_y, init_yaw)
        self._adopted_any_marker = False
        self._last_correction_log_t: Optional[Time] = None
        # divergence detection: if many high-quality measurements in a row
        # are rejected as outliers, the filter is wrong. Snap to recover.
        self._consec_outlier_count = 0
        self._last_rejected_measurement: Optional[Tuple[float, float, float]] = None

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ---- sub/pub ----
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.correction_norm_pub = self.create_publisher(Float64, '/aruco/correction_norm', 10)

        # ---- TF publish timer (decoupled from camera rate) ----
        self.create_timer(1.0 / max(tf_publish_rate, 1.0), self._publish_tf_and_pose)

        self.get_logger().info(
            f'aruco_node ready: side={self.marker_side:.2f}m dict={dict_name} '
            f'image={image_topic} info={camera_info_topic} '
            f'drift_correction={"on" if self.enable_drift_correction else "off"} '
            f'known_markers={len(self.marker_world_poses)} '
            f'init_pose=({init_x:.2f},{init_y:.2f},{init_yaw:.2f}) '
            f'alpha_xy={self.alpha_xy:.2f} alpha_yaw={self.alpha_yaw:.2f}'
        )

    # ---------- marker world poses ----------

    def _load_marker_world_poses(self, path: str) -> Dict[int, Dict[str, float]]:
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('kolestel_rover_description')
                path = os.path.join(pkg, 'config', 'aruco_world_poses.yaml')
            except Exception as exc:
                self.get_logger().warn(f'cannot resolve default marker poses path: {exc}')
                return {}

        if not os.path.isfile(path):
            self.get_logger().warn(f'marker_world_poses file not found at "{path}", drift correction disabled')
            self.enable_drift_correction = False
            return {}

        try:
            with open(path, encoding='utf-8') as fh:
                doc = yaml.safe_load(fh)
            poses_raw = doc.get('aruco_markers', {}) if isinstance(doc, dict) else {}
            out: Dict[int, Dict[str, float]] = {}
            for k, v in poses_raw.items():
                try:
                    mid = int(k)
                except (TypeError, ValueError):
                    continue
                if not isinstance(v, dict):
                    continue
                out[mid] = {
                    'x': float(v.get('x', 0.0)),
                    'y': float(v.get('y', 0.0)),
                    'z': float(v.get('z', 0.0)),
                    'yaw': float(v.get('yaw', 0.0)),
                }
            self.get_logger().info(f'loaded {len(out)} marker world poses from {path}')
            return out
        except Exception as exc:
            self.get_logger().error(f'failed to load marker world poses from {path}: {exc}')
            return {}

    # ---------- /odom -> predict step ----------

    def _odom_cb(self, msg: Odometry) -> None:
        T_OB_now = odom_pose_to_mat(msg)
        with self._lock:
            if self._prev_odom_mat is None:
                self._prev_odom_mat = T_OB_now
                self._latest_odom = msg
                return

            # delta_body = inv(T_OB_prev) * T_OB_now  -> body-frame motion since last tick
            delta_body = inv_se3(self._prev_odom_mat) @ T_OB_now
            # Apply that delta to the filter pose in the body frame
            self._T_WB = self._T_WB @ delta_body
            # keep planar state (drop any roll/pitch drift from the multiply)
            x, y, yaw = mat_to_planar(self._T_WB)
            self._T_WB = planar_mat(x, y, yaw)
            self._prev_odom_mat = T_OB_now
            self._latest_odom = msg

    # ---------- base -> camera_optical ----------

    def _get_base_to_camera_mat(self) -> Optional[np.ndarray]:
        if self._base_to_camera_mat is not None:
            return self._base_to_camera_mat
        try:
            ts = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if not self._base_to_camera_warned:
                self.get_logger().warn(
                    f'waiting for {self.base_frame} -> {self.camera_optical_frame} TF: {exc}'
                )
                self._base_to_camera_warned = True
            return None
        self._base_to_camera_mat = transform_msg_to_mat(ts)
        self.get_logger().info(
            f'cached static {self.base_frame} -> {self.camera_optical_frame}: '
            f'xyz=({ts.transform.translation.x:.3f},{ts.transform.translation.y:.3f},'
            f'{ts.transform.translation.z:.3f})'
        )
        return self._base_to_camera_mat

    # ---------- camera_info ----------

    def camera_info_cb(self, msg: CameraInfo) -> None:
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
    def _decode_image(msg: Image) -> Optional[np.ndarray]:
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

    # ---------- main image callback ----------

    def image_cb(self, msg: Image) -> None:
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
        # collect (T_WB_meas, weight) for valid detections in this frame
        frame_estimates: List[Tuple[np.ndarray, float]] = []

        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                pts = corners[i].reshape(4, 2)

                pts_int = pts.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(frame, [pts_int], True, (0, 255, 0), 2)

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

                self._draw_axes(frame, rvec, tvec, self.marker_side * 0.5)

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

                if not self.enable_drift_correction:
                    continue
                if distance < self.min_marker_distance_m or distance > self.max_marker_distance_m:
                    continue
                world_pose = self.marker_world_poses.get(int(marker_id))
                if world_pose is None:
                    continue

                # marker normal angle to camera ray: rotate marker_Z = (0,0,1) by R_CM
                # and compare to -tvec/|tvec| (the direction from marker to camera).
                R_cm, _ = cv2.Rodrigues(rvec)
                marker_z_in_cam = R_cm[:, 2]
                tnorm = tvec.flatten() / max(distance, 1e-6)
                # cos(angle) between marker_z_in_cam and -tnorm
                cos_ang = float(np.clip(-np.dot(marker_z_in_cam, tnorm), -1.0, 1.0))
                obliqueness = math.acos(cos_ang)
                if obliqueness > self.max_marker_obliqueness_rad:
                    continue

                T_WB_meas = self._compute_world_robot_pose(rvec, tvec, world_pose)
                if T_WB_meas is None:
                    continue

                weight = 1.0 / max(distance, 0.1)
                frame_estimates.append((T_WB_meas, weight))

        color = (0, 255, 0) if count > 0 else (160, 160, 160)
        cv2.putText(frame, f'markers detected: {count}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        if self.enable_drift_correction and frame_estimates:
            self._apply_measurement(frame_estimates)

        out = self._encode_image(frame, msg.header)
        self.debug_pub.publish(out)

    # ---------- fuser core ----------

    def _compute_world_robot_pose(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        marker_world: Dict[str, float],
    ) -> Optional[np.ndarray]:
        T_BC = self._get_base_to_camera_mat()
        if T_BC is None:
            return None

        T_CM = np.eye(4)
        R_cm, _ = cv2.Rodrigues(rvec)
        T_CM[0:3, 0:3] = R_cm
        T_CM[0:3, 3] = tvec.flatten()

        T_WM = make_marker_world_mat(
            marker_world['x'], marker_world['y'],
            marker_world['z'], marker_world['yaw'],
        )

        # T_WB = T_WM * inv(T_CM) * inv(T_BC)
        return T_WM @ inv_se3(T_CM) @ inv_se3(T_BC)

    def _apply_measurement(self, estimates: List[Tuple[np.ndarray, float]]) -> None:
        # 1) weighted-average xy/yaw across all markers in the current frame
        sum_w = 0.0
        sum_x = 0.0
        sum_y = 0.0
        sum_cos = 0.0
        sum_sin = 0.0
        for T, w in estimates:
            mx, my, myaw = mat_to_planar(T)
            sum_w += w
            sum_x += w * mx
            sum_y += w * my
            sum_cos += w * math.cos(myaw)
            sum_sin += w * math.sin(myaw)
        if sum_w <= 0:
            return
        meas_x = sum_x / sum_w
        meas_y = sum_y / sum_w
        meas_yaw = math.atan2(sum_sin / sum_w, sum_cos / sum_w)

        with self._lock:
            cur_x, cur_y, cur_yaw = mat_to_planar(self._T_WB)

            # 2) outlier rejection vs current filter
            d_xy = math.hypot(meas_x - cur_x, meas_y - cur_y)
            d_yaw = abs(shortest_angle(meas_yaw - cur_yaw))
            is_outlier = (
                self._adopted_any_marker and
                (d_xy > self.outlier_xy_m or d_yaw > self.outlier_yaw_rad)
            )
            if is_outlier:
                self._consec_outlier_count += 1
                self._last_rejected_measurement = (meas_x, meas_y, meas_yaw)
                # divergence recovery: if many high-quality measurements have
                # been rejected, the filter is the wrong one. Snap to it.
                if self._consec_outlier_count >= self.consec_outliers_for_resync:
                    self.get_logger().warn(
                        f'filter divergence detected after {self._consec_outlier_count} '
                        f'consecutive outliers; resyncing filter to last measurement '
                        f'({meas_x:.2f},{meas_y:.2f},{math.degrees(meas_yaw):.1f}deg)'
                    )
                    self._T_WB = planar_mat(meas_x, meas_y, meas_yaw)
                    self._consec_outlier_count = 0
                    self._last_rejected_measurement = None
                else:
                    self.get_logger().warn(
                        f'outlier rejected ({self._consec_outlier_count}/'
                        f'{self.consec_outliers_for_resync}): d_xy={d_xy:.2f}m '
                        f'd_yaw={math.degrees(d_yaw):.1f}deg',
                        throttle_duration_sec=2.0,
                    )
                return

            self._consec_outlier_count = 0
            self._last_rejected_measurement = None

            # 3) low-pass blend (planar lerp)
            new_x = (1 - self.alpha_xy) * cur_x + self.alpha_xy * meas_x
            new_y = (1 - self.alpha_xy) * cur_y + self.alpha_xy * meas_y
            new_yaw = cur_yaw + self.alpha_yaw * shortest_angle(meas_yaw - cur_yaw)
            self._T_WB = planar_mat(new_x, new_y, new_yaw)
            adopted_first = not self._adopted_any_marker
            self._adopted_any_marker = True

        # log + diagnostic publish (outside lock)
        if adopted_first:
            self.get_logger().info(
                f'first marker measurement applied: '
                f'meas=({meas_x:.2f},{meas_y:.2f},{math.degrees(meas_yaw):.1f}deg) '
                f'd_xy={d_xy:.2f}m d_yaw={math.degrees(d_yaw):.1f}deg'
            )
        now = self.get_clock().now()
        if (
            self._last_correction_log_t is None
            or (now - self._last_correction_log_t).nanoseconds > 2_000_000_000
        ):
            self._last_correction_log_t = now
            self.get_logger().info(
                f'fused pose: ({new_x:.2f},{new_y:.2f},{math.degrees(new_yaw):.1f}deg) '
                f'meas_offset={d_xy:.2f}m {math.degrees(d_yaw):.1f}deg '
                f'(n={len(estimates)})'
            )

        norm = Float64()
        norm.data = float(d_xy)
        self.correction_norm_pub.publish(norm)

    # ---------- TF + /aruco/pose output ----------

    def _publish_tf_and_pose(self) -> None:
        with self._lock:
            T_WB = self._T_WB.copy()
            T_OB = self._prev_odom_mat.copy() if self._prev_odom_mat is not None else None

        # If we haven't seen /odom yet, publish identity-correction (map==odom)
        # plus the initial pose as map->odom translation, so the TF tree is
        # consistent during startup.
        if T_OB is None:
            T_map_odom = T_WB.copy()
        else:
            T_map_odom = T_WB @ inv_se3(T_OB)

        # ---- map -> odom TF ----
        qx, qy, qz, qw = mat_to_quat(T_map_odom[0:3, 0:3])
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.map_frame
        ts.child_frame_id = self.odom_frame
        ts.transform.translation.x = float(T_map_odom[0, 3])
        ts.transform.translation.y = float(T_map_odom[1, 3])
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x = qx
        ts.transform.rotation.y = qy
        ts.transform.rotation.z = qz
        ts.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(ts)

        # ---- /aruco/pose (fused planar pose, for cv_navigator) ----
        x, y, yaw = mat_to_planar(T_WB)
        ps = PoseStamped()
        ps.header.stamp = ts.header.stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        qx2, qy2, qz2, qw2 = yaw_to_quat(yaw)
        ps.pose.orientation.x = qx2
        ps.pose.orientation.y = qy2
        ps.pose.orientation.z = qz2
        ps.pose.orientation.w = qw2
        self.pose_pub.publish(ps)

    # ---------- debug drawing ----------

    def _draw_axes(self, frame: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, length: float) -> None:
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


def main() -> None:
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
