#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ultralytics import YOLO
import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Float32MultiArray, Bool


TARGET_TIMEOUT_SEC = 2.0


# ==========================================
# HELPER FUNCTIONS
# ==========================================
def low_pass_filter(prev_value, new_value, alpha=0.7):
    if new_value is None:
        return prev_value
    if prev_value is None:
        return new_value
    return alpha * prev_value + (1.0 - alpha) * new_value


def normalize_error(u, v, cx, cy):
    ex = u - cx
    ey = v - cy

    ex_norm = ex / cx if cx != 0 else 0.0
    ey_norm = ey / cy if cy != 0 else 0.0

    ex_norm = float(np.clip(ex_norm, -1.0, 1.0))
    ey_norm = float(np.clip(ey_norm, -1.0, 1.0))

    return ex, ey, ex_norm, ey_norm


def xyxy_to_xywh(x1, y1, x2, y2):
    return (float(x1), float(y1), float(x2 - x1), float(y2 - y1))


def xywh_to_xyxy(x, y, w, h):
    return (float(x), float(y), float(x + w), float(y + h))


def clamp_bbox_xyxy(x1, y1, x2, y2, img_w, img_h):
    x1 = max(0, min(float(x1), img_w - 1))
    y1 = max(0, min(float(y1), img_h - 1))
    x2 = max(0, min(float(x2), img_w - 1))
    y2 = max(0, min(float(y2), img_h - 1))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return x1, y1, x2, y2


def bbox_center(x1, y1, x2, y2):
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


def bbox_size(x1, y1, x2, y2):
    return (x2 - x1, y2 - y1)


def bbox_area(x1, y1, x2, y2):
    w = max(0.0, x2 - x1)
    h = max(0.0, y2 - y1)
    return w * h


def compute_iou(boxA, boxB):
    ax1, ay1, ax2, ay2 = boxA
    bx1, by1, bx2, by2 = boxB

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih

    union = bbox_area(*boxA) + bbox_area(*boxB) - inter
    if union <= 1e-6:
        return 0.0
    return inter / union


def center_distance(boxA, boxB):
    ax, ay = bbox_center(*boxA)
    bx, by = bbox_center(*boxB)
    return math.hypot(ax - bx, ay - by)


def get_tracker(preferred="KCF"):
    preferred = preferred.upper()
    candidates = [preferred]

    for name in ["KCF", "CSRT", "MOSSE"]:
        if name not in candidates:
            candidates.append(name)

    for name in candidates:
        if name == "KCF":
            if hasattr(cv2, "TrackerKCF_create"):
                print("[INFO] Using tracker: KCF")
                return cv2.TrackerKCF_create()
            if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerKCF_create"):
                print("[INFO] Using tracker: legacy.KCF")
                return cv2.legacy.TrackerKCF_create()

        elif name == "CSRT":
            if hasattr(cv2, "TrackerCSRT_create"):
                print("[INFO] Using tracker: CSRT")
                return cv2.TrackerCSRT_create()
            if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
                print("[INFO] Using tracker: legacy.CSRT")
                return cv2.legacy.TrackerCSRT_create()

        elif name == "MOSSE":
            if hasattr(cv2, "TrackerMOSSE_create"):
                print("[INFO] Using tracker: MOSSE")
                return cv2.TrackerMOSSE_create()
            if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
                print("[INFO] Using tracker: legacy.MOSSE")
                return cv2.legacy.TrackerMOSSE_create()

    raise RuntimeError(
        "No OpenCV tracker available in this build. "
        "Install opencv-contrib-python:\n"
        "pip uninstall -y opencv-python opencv-contrib-python "
        "opencv-python-headless opencv-contrib-python-headless\n"
        "pip install opencv-contrib-python"
    )


# ==========================================
# NODE
# ==========================================
class PersonFollowNode(Node):
    def __init__(self):
        super().__init__("person_follow_node")

        # =========================
        # PARAMETERS
        # =========================
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("click_point_topic", "/click_point")
        self.declare_parameter("select_bbox_topic", "/select_bbox")
        self.declare_parameter("debug_image_topic", "/image_proc")
        self.declare_parameter("target_pose_topic", "/detecd_pose")
        self.declare_parameter("reset_topic", "/follow_reset")
        self.declare_parameter("tag_state_topic", "/tag_state")
        self.declare_parameter("ai_output_topic", "/ai_output")

        self.declare_parameter("model_path", "models/yolov8n.pt")
        self.declare_parameter("conf_thres", 0.30)
        self.declare_parameter("person_class_id", 0)

        self.declare_parameter("resize_width", 1280)
        self.declare_parameter("resize_height", 720)
        self.declare_parameter("detect_width", 640)
        self.declare_parameter("detect_height", 384)

        self.declare_parameter("detect_every_n_frames", 6)
        self.declare_parameter("redetect_every_n_frames", 12)
        self.declare_parameter("max_missed_redetect", 4)

        self.declare_parameter("tracker_type", "KCF")
        self.declare_parameter("use_grayscale_for_tracking", True)

        self.declare_parameter("max_center_jump_px", 180.0)
        self.declare_parameter("min_bbox_w", 12)
        self.declare_parameter("min_bbox_h", 20)

        self.declare_parameter("z_mode", "pinhole")
        self.declare_parameter("target_real_height_m", 1.70)
        self.declare_parameter("ref_bbox_height", 200.0)

        self.declare_parameter("alpha_uv", 0.7)
        self.declare_parameter("alpha_z", 0.8)

        self.declare_parameter("enable_gui", True)
        self.declare_parameter("window_name", "Person Follow - Detector + Tracker")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("show_debug_text", True)

        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("select_iou_threshold", 0.1)

        self.declare_parameter("target_valid_topic", "/target_valid")
        self.declare_parameter("target_bbox_topic", "/target_bbox")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.click_point_topic = self.get_parameter("click_point_topic").value
        self.select_bbox_topic = self.get_parameter("select_bbox_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.reset_topic = self.get_parameter("reset_topic").value
        self.tag_state_topic = self.get_parameter("tag_state_topic").value
        self.ai_output_topic = self.get_parameter("ai_output_topic").value
        self.target_valid_topic = self.get_parameter("target_valid_topic").value
        self.target_bbox_topic = self.get_parameter("target_bbox_topic").value

        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.person_class_id = int(self.get_parameter("person_class_id").value)

        self.resize_width = int(self.get_parameter("resize_width").value)
        self.resize_height = int(self.get_parameter("resize_height").value)
        self.detect_width = int(self.get_parameter("detect_width").value)
        self.detect_height = int(self.get_parameter("detect_height").value)

        self.detect_every_n_frames = int(self.get_parameter("detect_every_n_frames").value)
        self.redetect_every_n_frames = int(self.get_parameter("redetect_every_n_frames").value)
        self.max_missed_redetect = int(self.get_parameter("max_missed_redetect").value)

        self.tracker_type = self.get_parameter("tracker_type").value
        self.use_grayscale_for_tracking = bool(
            self.get_parameter("use_grayscale_for_tracking").value
        )

        self.max_center_jump_px = float(self.get_parameter("max_center_jump_px").value)
        self.min_bbox_w = int(self.get_parameter("min_bbox_w").value)
        self.min_bbox_h = int(self.get_parameter("min_bbox_h").value)

        self.z_mode = self.get_parameter("z_mode").value
        self.target_real_height_m = float(self.get_parameter("target_real_height_m").value)
        self.ref_bbox_height = float(self.get_parameter("ref_bbox_height").value)

        self.alpha_uv = float(self.get_parameter("alpha_uv").value)
        self.alpha_z = float(self.get_parameter("alpha_z").value)

        self.enable_gui = bool(self.get_parameter("enable_gui").value)
        self.window_name = self.get_parameter("window_name").value
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.show_debug_text = bool(self.get_parameter("show_debug_text").value)

        self.output_frame_id = self.get_parameter("frame_id").value
        self.select_iou_threshold = float(self.get_parameter("select_iou_threshold").value)

        # =========================
        # QOS
        # =========================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =========================
        # STATE
        # =========================
        self.bridge = CvBridge()

        self.tracker = None
        self.locked = False
        self.locked_bbox_xyxy = None
        self.last_person_dets = []

        self.mouse_click = None
        self.pending_select_bbox = None

        self.frame_count = 0
        self.last_detect_frame = -999
        self.last_redetect_frame = -999
        self.missed_redetect_count = 0

        self.filtered_u = None
        self.filtered_v = None
        self.filtered_z = None
        self.last_track_center = None

        self.has_valid_pose = False
        self.last_seen_time = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.dist_coeffs = None
        self.camera_width = None
        self.camera_height = None
        self.camera_info_received = False

        # =========================
        # MODEL
        # =========================
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # =========================
        # ROS I/O
        # =========================
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos
        )

        self.click_point_sub = self.create_subscription(
            PointStamped,
            self.click_point_topic,
            self.click_point_callback,
            QoSProfile(depth=10),
        )

        self.select_bbox_sub = self.create_subscription(
            Float32MultiArray,
            self.select_bbox_topic,
            self.select_bbox_callback,
            QoSProfile(depth=10),
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, pub_qos
        )

        self.target_valid_pub = self.create_publisher(
            Bool, self.target_valid_topic, QoSProfile(depth=10)
        )

        self.target_bbox_pub = self.create_publisher(
            Float32MultiArray, self.target_bbox_topic, QoSProfile(depth=10)
        )

        self.ai_output_pub = self.create_publisher(
            Float32MultiArray, self.ai_output_topic, QoSProfile(depth=10)
        )

        self.reset_pub = self.create_publisher(
            String, self.reset_topic, sensor_qos
        )

        self.tag_state_pub = self.create_publisher(
            String, self.tag_state_topic, QoSProfile(depth=10)
        )

        self.image_pub = self.create_publisher(
            Image, self.debug_image_topic, sensor_qos
        )

        # =========================
        # GUI
        # =========================
        if self.enable_gui:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.resize_width, self.resize_height)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info(f"Subscribed image topic: {self.image_topic}")
        self.get_logger().info(f"Subscribed camera_info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Subscribed click topic: {self.click_point_topic}")
        self.get_logger().info(f"Subscribed select bbox topic: {self.select_bbox_topic}")
        self.get_logger().info(f"Publishing target pose: {self.target_pose_topic}")
        self.get_logger().info(f"Publishing AI output: {self.ai_output_topic}")

    # =========================================================
    # CAMERA INFO
    # =========================================================
    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.camera_width = int(msg.width)
        self.camera_height = int(msg.height)
        self.camera_info_received = True

        self.get_logger().info(
            f"CameraInfo received: "
            f"fx={self.fx:.3f}, fy={self.fy:.3f}, "
            f"cx={self.cx:.3f}, cy={self.cy:.3f}, "
            f"size=({self.camera_width}x{self.camera_height})"
        )

        if self.camera_info_sub is not None:
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None

    def get_scaled_intrinsics(self):
        if not self.camera_info_received:
            return None, None, None, None

        if self.camera_width is None or self.camera_height is None:
            return None, None, None, None

        sx = self.resize_width / float(self.camera_width)
        sy = self.resize_height / float(self.camera_height)

        fx = self.fx * sx
        fy = self.fy * sy
        cx = self.cx * sx
        cy = self.cy * sy
        return fx, fy, cx, cy

    def estimate_distance(self, bbox_height_px):
        if bbox_height_px <= 1:
            return None

        if self.z_mode == "pinhole":
            _, fy, _, _ = self.get_scaled_intrinsics()
            if fy is None:
                return None
            return float((fy * self.target_real_height_m) / bbox_height_px)

        if self.z_mode == "relative":
            return float(self.ref_bbox_height / bbox_height_px)

        return None

    def pixel_to_camera_xyz(self, u, v, z):
        fx, fy, cx, cy = self.get_scaled_intrinsics()
        if fx is None or fy is None or cx is None or cy is None:
            return None, None, None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return float(x), float(y), float(z)

    # =========================================================
    # INPUT CALLBACKS
    # =========================================================
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_click = (x, y)

    def click_point_callback(self, msg: PointStamped):
        x = int(msg.point.x)
        y = int(msg.point.y)
        self.mouse_click = (x, y)
        self.get_logger().info(f"Received external click: ({x}, {y})")

    def select_bbox_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("select_bbox requires [x, y, w, h]")
            return

        x = float(msg.data[0])
        y = float(msg.data[1])
        w = float(msg.data[2])
        h = float(msg.data[3])

        self.pending_select_bbox = [x, y, x + w, y + h]
        self.get_logger().info(
            f"Received select_bbox: x={x:.1f}, y={y:.1f}, w={w:.1f}, h={h:.1f}"
        )

    # =========================================================
    # DETECTOR / TRACKER
    # =========================================================
    def detect_persons(self, frame_bgr):
        small = cv2.resize(frame_bgr, (self.detect_width, self.detect_height))
        scale_x = frame_bgr.shape[1] / float(self.detect_width)
        scale_y = frame_bgr.shape[0] / float(self.detect_height)

        results = self.model(
            small,
            conf=self.conf_thres,
            classes=[self.person_class_id],
            verbose=False,
        )[0]

        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])

            x1 *= scale_x
            x2 *= scale_x
            y1 *= scale_y
            y2 *= scale_y

            x1, y1, x2, y2 = clamp_bbox_xyxy(
                x1, y1, x2, y2, frame_bgr.shape[1], frame_bgr.shape[0]
            )

            w, h = bbox_size(x1, y1, x2, y2)
            if w < self.min_bbox_w or h < self.min_bbox_h:
                continue

            detections.append({"bbox": (x1, y1, x2, y2), "conf": conf})

        return detections

    def init_tracker(self, frame_bgr, bbox_xyxy):
        x1, y1, x2, y2 = bbox_xyxy
        x, y, w, h = xyxy_to_xywh(x1, y1, x2, y2)

        if w < self.min_bbox_w or h < self.min_bbox_h:
            self.get_logger().warn("BBox too small, skip tracker init")
            return False

        x = int(round(x))
        y = int(round(y))
        w = int(round(w))
        h = int(round(h))

        if w <= 0 or h <= 0:
            self.get_logger().warn("Invalid bbox for tracker init")
            return False

        try:
            self.tracker = get_tracker(self.tracker_type)
        except Exception as e:
            self.tracker = None
            self.locked = False
            self.get_logger().error(
                f"Cannot initialize tracker '{self.tracker_type}': {e}"
            )
            return False

        init_frame = frame_bgr
        if self.use_grayscale_for_tracking:
            init_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        bbox = (x, y, w, h)

        try:
            ok = self.tracker.init(init_frame, bbox)
        except Exception as e:
            self.tracker = None
            self.locked = False
            self.get_logger().error(f"Tracker init() failed: {e}")
            return False

        if ok is None:
            ok = True

        if not ok:
            self.tracker = None
            self.locked = False
            self.get_logger().warn("Tracker init returned False")
            return False

        self.locked_bbox_xyxy = (float(x), float(y), float(x + w), float(y + h))
        self.locked = True
        self.missed_redetect_count = 0
        self.filtered_u = None
        self.filtered_v = None
        self.filtered_z = None
        self.last_track_center = bbox_center(*self.locked_bbox_xyxy)

        self.get_logger().info(
            f"Tracker initialized successfully: {self.tracker_type}, bbox=({x},{y},{w},{h})"
        )
        return True

    def update_tracker(self, frame_bgr):
        if self.tracker is None:
            return False, None

        track_frame = frame_bgr
        if self.use_grayscale_for_tracking:
            track_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        ok, box = self.tracker.update(track_frame)
        if not ok:
            return False, None

        x, y, w, h = box
        x = float(x)
        y = float(y)
        w = float(w)
        h = float(h)

        x1, y1, x2, y2 = xywh_to_xyxy(x, y, w, h)
        x1, y1, x2, y2 = clamp_bbox_xyxy(
            x1, y1, x2, y2, frame_bgr.shape[1], frame_bgr.shape[0]
        )

        w2, h2 = bbox_size(x1, y1, x2, y2)
        if w2 < self.min_bbox_w or h2 < self.min_bbox_h:
            return False, None

        curr_center = bbox_center(x1, y1, x2, y2)
        if self.last_track_center is not None:
            jump = math.hypot(
                curr_center[0] - self.last_track_center[0],
                curr_center[1] - self.last_track_center[1],
            )
            if jump > self.max_center_jump_px:
                return False, None

        self.last_track_center = curr_center
        return True, (x1, y1, x2, y2)

    def find_detection_from_click(self, click_xy):
        mx, my = click_xy
        best_det = None
        best_area = 1e18

        for det in self.last_person_dets:
            x1, y1, x2, y2 = det["bbox"]
            if x1 <= mx <= x2 and y1 <= my <= y2:
                area = bbox_area(x1, y1, x2, y2)
                if area < best_area:
                    best_det = det
                    best_area = area

        return best_det


    def lock_from_click(self, frame):
        if self.mouse_click is None:
            return

        det = self.find_detection_from_click(self.mouse_click)

        if det is None:
            self.get_logger().warn(
                f"Click {self.mouse_click} did not hit any detected person"
            )
            self.mouse_click = None
            return

        ok = self.init_tracker(frame, det["bbox"])
        if ok:
            self.get_logger().info(
                f"Locked target from click at {self.mouse_click}"
            )
        else:
            self.get_logger().warning(
                "Tracker unavailable or failed to initialize from click"
            )

        self.mouse_click = None

    def match_redetection(self, current_bbox, detections):
        if current_bbox is None or len(detections) == 0:
            return None

        best_det = None
        best_score = -1e9

        for det in detections:
            det_bbox = det["bbox"]
            iou = compute_iou(current_bbox, det_bbox)
            dist = center_distance(current_bbox, det_bbox)
            conf = det["conf"]

            score = 3.0 * iou + 0.3 * conf - 0.002 * dist

            if score > best_score:
                best_score = score
                best_det = det

        if best_det is None:
            return None

        iou = compute_iou(current_bbox, best_det["bbox"])
        dist = center_distance(current_bbox, best_det["bbox"])

        if iou < 0.05 and dist > 140.0:
            return None

        return best_det

    def bbox_iou(self, box_a, box_b):
        return compute_iou(box_a, box_b)

    # =========================================================
    # PUBLISHERS
    # =========================================================
    def publish_pose(self, stamp, x_cam, y_cam, z_cam):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.output_frame_id

        pose_msg.pose.position.x = float(x_cam)
        pose_msg.pose.position.y = float(y_cam)
        pose_msg.pose.position.z = float(z_cam)

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.target_pose_pub.publish(pose_msg)

    def publish_target_valid(self, valid: bool):
        msg = Bool()
        msg.data = bool(valid)
        self.target_valid_pub.publish(msg)

    def publish_target_bbox(self, u=0.0, v=0.0, bw=0.0, bh=0.0, x_out=0.0, y_out=0.0, z_out=0.0):
        arr = Float32MultiArray()
        arr.data = [
            float(u),
            float(v),
            float(bw),
            float(bh),
            float(x_out),
            float(y_out),
            float(z_out if z_out is not None else -1.0),
        ]
        self.target_bbox_pub.publish(arr)

    def publish_ai_output(
        self,
        target_valid=False,
        locked=False,
        u=-1.0,
        v=-1.0,
        bw=-1.0,
        bh=-1.0,
        x_out=-1.0,
        y_out=-1.0,
        z_out=-1.0,
        x1=-1.0,
        y1=-1.0,
        x2=-1.0,
        y2=-1.0,
    ):
        msg = Float32MultiArray()
        msg.data = [
            1.0 if target_valid else 0.0,
            1.0 if locked else 0.0,
            float(u),
            float(v),
            float(bw),
            float(bh),
            float(x_out),
            float(y_out),
            float(z_out if z_out is not None else -1.0),
            float(x1),
            float(y1),
            float(x2),
            float(y2),
        ]
        self.ai_output_pub.publish(msg)

    def publish_state(self, found, stamp):
        state_msg = String()
        tag_state_msg = String()

        if found:
            state_msg.data = "ACTIVE"
            self.reset_pub.publish(state_msg)

            tag_state_msg.data = "1"
            self.tag_state_pub.publish(tag_state_msg)
            return

        tag_state_msg.data = "0"
        self.tag_state_pub.publish(tag_state_msg)

        now_sec = stamp.sec + stamp.nanosec * 1e-9

        if self.has_valid_pose and self.last_seen_time is not None:
            dt = now_sec - self.last_seen_time
            if dt > TARGET_TIMEOUT_SEC:
                state_msg.data = "RESET"
                self.reset_pub.publish(state_msg)
                self.has_valid_pose = False
        else:
            state_msg.data = "RESET"
            self.reset_pub.publish(state_msg)

    def reset_target(self):
        self.locked = False
        self.tracker = None
        self.locked_bbox_xyxy = None
        self.missed_redetect_count = 0
        self.filtered_u = None
        self.filtered_v = None
        self.filtered_z = None
        self.last_track_center = None
        self.has_valid_pose = False
        self.last_seen_time = None
        self.pending_select_bbox = None
        self.get_logger().info("Target reset")

    # =========================================================
    # MAIN IMAGE CALLBACK
    # =========================================================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        frame = cv2.resize(frame, (self.resize_width, self.resize_height))
        display = frame.copy()

        img_h, img_w = frame.shape[:2]
        cx_img = img_w // 2
        cy_img = img_h // 2

        self.frame_count += 1

        cv2.circle(display, (cx_img, cy_img), 5, (0, 255, 255), -1)
        cv2.line(display, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (0, 255, 255), 2)
        cv2.line(display, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (0, 255, 255), 2)

        if not self.camera_info_received:
            cv2.putText(
                display,
                "Waiting for /camera/camera_info ...",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        force_detect_for_click = (
            (not self.locked)
            and (self.mouse_click is not None or self.pending_select_bbox is not None)
        )

        do_detect = (
            force_detect_for_click
            or
            (
                (not self.locked and (self.frame_count - self.last_detect_frame >= self.detect_every_n_frames))
                or
                (self.locked and (self.frame_count - self.last_redetect_frame >= self.redetect_every_n_frames))
            )
        )

        if do_detect:
            persons = self.detect_persons(frame)
            self.last_person_dets = persons

            if not self.locked:
                self.last_detect_frame = self.frame_count
            else:
                self.last_redetect_frame = self.frame_count

                matched = self.match_redetection(self.locked_bbox_xyxy, persons)
                if matched is not None:
                    ok = self.init_tracker(frame, matched["bbox"])
                    if ok:
                        self.missed_redetect_count = 0
                    else:
                        self.missed_redetect_count += 1
                else:
                    self.missed_redetect_count += 1

                if self.missed_redetect_count >= self.max_missed_redetect:
                    self.get_logger().warning("Target lost after repeated redetect failures")
                    self.publish_ai_output(
                        target_valid=False,
                        locked=False
                    )
                    self.reset_target()

        found = False
        target_found = False

        if not self.locked:
            for det in self.last_person_dets:
                x1, y1, x2, y2 = map(int, det["bbox"])
                conf = det["conf"]

                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    display,
                    f"person {conf:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

            if self.mouse_click is not None:
                self.lock_from_click(frame)

            if self.pending_select_bbox is not None:
                best_det = None
                best_iou = 0.0

                for det in self.last_person_dets:
                    iou = self.bbox_iou(self.pending_select_bbox, det["bbox"])
                    if iou > best_iou:
                        best_iou = iou
                        best_det = det

                if best_det is not None and best_iou >= self.select_iou_threshold:
                    ok = self.init_tracker(frame, best_det["bbox"])
                    if ok:
                        self.get_logger().info(
                            f"Locked target from select_bbox, IoU={best_iou:.3f}"
                        )
                    else:
                        self.get_logger().warning(
                            "Tracker unavailable or failed to initialize from select_bbox"
                        )
                else:
                    self.get_logger().warn(
                        f"No detection matched select_bbox well enough. best_iou={best_iou:.3f}"
                    )

                self.pending_select_bbox = None

            cv2.putText(
                display,
                "State: DETECT / Click person to lock",
                (30, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            self.publish_target_valid(False)
            self.publish_target_bbox(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
            self.publish_ai_output(
                target_valid=False,
                locked=False
            )

        else:
            ok, tracked_bbox = self.update_tracker(frame)

            if not ok or tracked_bbox is None:
                cv2.putText(
                    display,
                    "Tracker lost - waiting redetect",
                    (30, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )
                self.publish_target_valid(False)
                self.publish_target_bbox(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
                self.publish_ai_output(
                    target_valid=False,
                    locked=True
                )

            else:
                found = True
                target_found = True

                self.locked_bbox_xyxy = tracked_bbox
                x1, y1, x2, y2 = tracked_bbox
                bw, bh = bbox_size(x1, y1, x2, y2)

                u = (x1 + x2) * 0.5
                v = (y1 + y2) * 0.5

                self.filtered_u = low_pass_filter(self.filtered_u, u, self.alpha_uv)
                self.filtered_v = low_pass_filter(self.filtered_v, v, self.alpha_uv)

                z_raw = self.estimate_distance(bh)
                self.filtered_z = low_pass_filter(self.filtered_z, z_raw, self.alpha_z)

                ex, ey, ex_norm, ey_norm = normalize_error(
                    self.filtered_u, self.filtered_v, cx_img, cy_img
                )

                x_out = ex_norm
                y_out = ey_norm
                z_out = self.filtered_z

                pose_x = x_out
                pose_y = y_out
                pose_z = z_out if z_out is not None else -1.0

                if self.filtered_z is not None and self.camera_info_received:
                    x_cam_m, y_cam_m, z_cam_m = self.pixel_to_camera_xyz(
                        self.filtered_u, self.filtered_v, self.filtered_z
                    )
                    if x_cam_m is not None:
                        pose_x = x_cam_m
                        pose_y = y_cam_m
                        pose_z = z_cam_m

                self.publish_pose(msg.header.stamp, pose_x, pose_y, pose_z)
                self.publish_target_valid(True)
                self.publish_target_bbox(
                    self.filtered_u,
                    self.filtered_v,
                    bw,
                    bh,
                    x_out,
                    y_out,
                    z_out if z_out is not None else -1.0,
                )
                self.publish_ai_output(
                    target_valid=True,
                    locked=True,
                    u=self.filtered_u,
                    v=self.filtered_v,
                    bw=bw,
                    bh=bh,
                    x_out=x_out,
                    y_out=y_out,
                    z_out=z_out if z_out is not None else -1.0,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )

                self.has_valid_pose = True
                self.last_seen_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

                xi1, yi1, xi2, yi2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(display, (xi1, yi1), (xi2, yi2), (0, 0, 255), 2)
                cv2.putText(
                    display,
                    "LOCKED person",
                    (xi1, max(20, yi1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )

                tu = int(self.filtered_u)
                tv = int(self.filtered_v)
                cv2.circle(display, (tu, tv), 6, (255, 0, 0), -1)
                cv2.line(display, (cx_img, cy_img), (tu, tv), (255, 255, 0), 2)

                if self.show_debug_text:
                    lines = [
                        f"State: TRACK ({self.tracker_type}, gray={self.use_grayscale_for_tracking})",
                        f"u, v = ({self.filtered_u:.1f}, {self.filtered_v:.1f}) px",
                        f"bbox_w, bbox_h = ({bw:.1f}, {bh:.1f})",
                        f"ex_norm, ey_norm = ({x_out:.3f}, {y_out:.3f})",
                        f"missed_redetect = {self.missed_redetect_count}",
                    ]

                    if self.camera_info_received and self.filtered_z is not None:
                        lines.append(f"Pose(m) = ({pose_x:.3f}, {pose_y:.3f}, {pose_z:.3f})")
                    else:
                        lines.append(f"Pose(out) = ({pose_x:.3f}, {pose_y:.3f}, {pose_z:.3f})")

                    if z_out is not None:
                        if self.z_mode == "pinhole":
                            lines.append(f"z_est = {z_out:.3f} m")
                        else:
                            lines.append(f"z_est = {z_out:.3f} (relative)")
                    else:
                        lines.append("z_est = None")

                    y0 = 30
                    for line in lines:
                        cv2.putText(
                            display,
                            line,
                            (30, y0),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (255, 255, 255),
                            2,
                        )
                        y0 += 28

        if self.locked and not target_found:
            cv2.putText(
                display,
                "Locked target lost",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        self.publish_state(found, msg.header.stamp)

        if self.publish_debug_image:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(display, encoding="bgr8")
                debug_msg.header = msg.header
                self.image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f"Cannot publish debug image: {e}")

        if self.enable_gui:
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                self.reset_target()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.enable_gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()