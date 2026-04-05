#!/usr/bin/env python3

import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Float32MultiArray


TARGET_TIMEOUT_SEC = 2.0


class ObjectLockTrackerNode(Node):
    def __init__(self):
        super().__init__("object_lock_tracker_node")

        # =========================
        # PARAMETERS
        # =========================
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("click_point_topic", "/click_point")
        self.declare_parameter("select_bbox_topic", "/select_bbox")
        self.declare_parameter("debug_image_topic", "/image_proc")
        self.declare_parameter("target_pose_topic", "/detecd_pose")
        self.declare_parameter("reset_topic", "/reset")
        self.declare_parameter("tag_state_topic", "/tag_state")

        self.declare_parameter("model_path", "models/yolov8n.pt")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("resize_width", 1280)
        self.declare_parameter("resize_height", 720)

        # z_mode:
        #   pinhole  -> Z = f * H / h
        #   relative -> Z tương đối, không phải mét thật
        #   none     -> không tính Z
        self.declare_parameter("z_mode", "pinhole")
        self.declare_parameter("target_real_height_m", 1.70)
        self.declare_parameter("ref_bbox_height", 200.0)

        self.declare_parameter("alpha_uv", 0.7)
        self.declare_parameter("alpha_z", 0.8)

        self.declare_parameter("enable_gui", True)
        self.declare_parameter("window_name", "Object Lock Tracker")
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("target_class_id", -1)  # -1 = all classes
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("select_iou_threshold", 0.1)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.click_point_topic = self.get_parameter("click_point_topic").value
        self.select_bbox_topic = self.get_parameter("select_bbox_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.reset_topic = self.get_parameter("reset_topic").value
        self.tag_state_topic = self.get_parameter("tag_state_topic").value

        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.resize_width = int(self.get_parameter("resize_width").value)
        self.resize_height = int(self.get_parameter("resize_height").value)

        self.z_mode = self.get_parameter("z_mode").value
        self.target_real_height_m = float(self.get_parameter("target_real_height_m").value)
        self.ref_bbox_height = float(self.get_parameter("ref_bbox_height").value)

        self.alpha_uv = float(self.get_parameter("alpha_uv").value)
        self.alpha_z = float(self.get_parameter("alpha_z").value)

        self.enable_gui = bool(self.get_parameter("enable_gui").value)
        self.window_name = self.get_parameter("window_name").value
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.output_frame_id = self.get_parameter("frame_id").value
        self.select_iou_threshold = float(self.get_parameter("select_iou_threshold").value)

        # =========================
        # QoS
        # =========================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =========================
        # STATE
        # =========================
        self.bridge = CvBridge()

        self.locked_id = None
        self.mouse_click = None
        self.pending_select_bbox = None  # [x1, y1, x2, y2]

        self.filtered_u = None
        self.filtered_v = None
        self.filtered_z = None

        self.has_valid_pose = False
        self.last_seen_time = None

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.dist_coeffs = None
        self.camera_width = None
        self.camera_height = None
        self.camera_info_received = False

        # =========================
        # MODEL + TRACKER
        # =========================
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        self.get_logger().info("Initializing ByteTrack")
        self.tracker = sv.ByteTrack()

        # =========================
        # ROS I/O
        # =========================
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        self.click_point_sub = self.create_subscription(
            PointStamped,
            self.click_point_topic,
            self.click_point_callback,
            QoSProfile(depth=10)
        )

        self.select_bbox_sub = self.create_subscription(
            Float32MultiArray,
            self.select_bbox_topic,
            self.select_bbox_callback,
            QoSProfile(depth=10)
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            self.target_pose_topic,
            pub_qos
        )

        self.reset_pub = self.create_publisher(
            String,
            self.reset_topic,
            sensor_qos
        )

        self.tag_state_pub = self.create_publisher(
            String,
            self.tag_state_topic,
            QoSProfile(depth=10)
        )

        self.image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            sensor_qos
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

    # =========================================================
    # CAMERA INFO CALLBACK
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

    # =========================================================
    # MOUSE CALLBACK
    # =========================================================
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_click = (x, y)

    # =========================================================
    # EXTERNAL CLICK CALLBACK
    # =========================================================
    def click_point_callback(self, msg: PointStamped):
        x = int(msg.point.x)
        y = int(msg.point.y)
        self.mouse_click = (x, y)
        self.get_logger().info(f"Received external click: ({x}, {y})")

    # =========================================================
    # EXTERNAL ROI CALLBACK
    # =========================================================
    def select_bbox_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("select_bbox requires [x, y, w, h]")
            return

        x = float(msg.data[0])
        y = float(msg.data[1])
        w = float(msg.data[2])
        h = float(msg.data[3])

        x1 = x
        y1 = y
        x2 = x + w
        y2 = y + h

        self.pending_select_bbox = [x1, y1, x2, y2]
        self.get_logger().info(
            f"Received select_bbox: x={x:.1f}, y={y:.1f}, w={w:.1f}, h={h:.1f}"
        )

    # =========================================================
    # HELPERS
    # =========================================================
    def low_pass_filter(self, prev_value, new_value, alpha):
        if prev_value is None:
            return new_value
        return alpha * prev_value + (1.0 - alpha) * new_value

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

            z_est = (fy * self.target_real_height_m) / bbox_height_px
            return float(z_est)

        if self.z_mode == "relative":
            z_est = self.ref_bbox_height / bbox_height_px
            return float(z_est)

        return None

    def pixel_to_camera_xyz(self, u, v, z):
        fx, fy, cx, cy = self.get_scaled_intrinsics()
        if fx is None or fy is None or cx is None or cy is None:
            return None, None, None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return float(x), float(y), float(z)

    def bbox_iou(self, box_a, box_b):
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)

        union_area = area_a + area_b - inter_area
        if union_area <= 1e-6:
            return 0.0

        return inter_area / union_area

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
                2
            )

        # =============================
        # YOLO DETECTION
        # =============================
        results = self.model(frame, conf=self.conf_thres, verbose=False)[0]

        boxes = []
        confidences = []
        class_ids = []

        if results.boxes is not None:
            for box in results.boxes:
                cls = int(box.cls[0])
                if self.target_class_id >= 0 and cls != self.target_class_id:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])

                boxes.append([x1, y1, x2, y2])
                confidences.append(conf)
                class_ids.append(cls)

        found = False
        target_found = False

        if len(boxes) > 0:
            detections = sv.Detections(
                xyxy=np.array(boxes),
                confidence=np.array(confidences),
                class_id=np.array(class_ids),
            )

            tracked = self.tracker.update_with_detections(detections)

            if tracked is not None and len(tracked) > 0:
                # ==========================================
                # HANDLE CLICK -> LOCK TARGET ID
                # ==========================================
                if self.mouse_click is not None:
                    mx, my = self.mouse_click
                    for xyxy, tid in zip(tracked.xyxy, tracked.tracker_id):
                        x1, y1, x2, y2 = xyxy
                        if x1 <= mx <= x2 and y1 <= my <= y2:
                            self.locked_id = int(tid)
                            self.filtered_u = None
                            self.filtered_v = None
                            self.filtered_z = None
                            self.get_logger().info(f"Locked ID from click: {self.locked_id}")
                            break
                    self.mouse_click = None

                # ==========================================
                # HANDLE ROI -> LOCK TARGET ID
                # ==========================================
                if self.pending_select_bbox is not None:
                    best_tid = None
                    best_iou = 0.0

                    for xyxy, tid in zip(tracked.xyxy, tracked.tracker_id):
                        iou = self.bbox_iou(self.pending_select_bbox, xyxy)
                        if iou > best_iou:
                            best_iou = iou
                            best_tid = int(tid)

                    if best_tid is not None and best_iou >= self.select_iou_threshold:
                        self.locked_id = best_tid
                        self.filtered_u = None
                        self.filtered_v = None
                        self.filtered_z = None
                        self.get_logger().info(
                            f"Locked ID from select_bbox: {self.locked_id}, IoU={best_iou:.3f}"
                        )
                    else:
                        self.get_logger().warn(
                            f"No track matched select_bbox well enough. best_iou={best_iou:.3f}"
                        )

                    self.pending_select_bbox = None

                # ==========================================
                # DRAW ALL TRACKS
                # ==========================================
                for xyxy, tid, cls_id in zip(tracked.xyxy, tracked.tracker_id, tracked.class_id):
                    x1, y1, x2, y2 = map(int, xyxy)
                    class_name = self.model.names[int(cls_id)]

                    if self.locked_id == int(tid):
                        color = (0, 0, 255)
                        label = f"{class_name} | LOCKED ID {int(tid)}"
                    else:
                        color = (0, 255, 0)
                        label = f"{class_name} | ID {int(tid)}"

                    cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        display,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )

                # Draw pending bbox if exists (for debug)
                if self.pending_select_bbox is not None:
                    x1, y1, x2, y2 = map(int, self.pending_select_bbox)
                    cv2.rectangle(display, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    cv2.putText(
                        display,
                        "PENDING ROI",
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 0, 255),
                        2,
                    )

                # ==========================================
                # PROCESS LOCKED TARGET
                # ==========================================
                if self.locked_id is not None:
                    for xyxy, tid, cls_id in zip(tracked.xyxy, tracked.tracker_id, tracked.class_id):
                        if int(tid) != self.locked_id:
                            continue

                        found = True
                        target_found = True

                        x1, y1, x2, y2 = map(int, xyxy)
                        bw = x2 - x1
                        bh = y2 - y1

                        u = (x1 + x2) / 2.0
                        v = (y1 + y2) / 2.0

                        self.filtered_u = self.low_pass_filter(self.filtered_u, u, self.alpha_uv)
                        self.filtered_v = self.low_pass_filter(self.filtered_v, v, self.alpha_uv)

                        z_est_raw = self.estimate_distance(bh)
                        if z_est_raw is not None:
                            self.filtered_z = self.low_pass_filter(self.filtered_z, z_est_raw, self.alpha_z)
                        else:
                            self.filtered_z = None

                        x_cam = None
                        y_cam = None
                        z_cam = None

                        if self.filtered_z is not None and self.camera_info_received:
                            x_cam, y_cam, z_cam = self.pixel_to_camera_xyz(
                                self.filtered_u,
                                self.filtered_v,
                                self.filtered_z
                            )

                            if x_cam is not None:
                                self.publish_pose(msg.header.stamp, x_cam, y_cam, z_cam)
                                self.has_valid_pose = True
                                self.last_seen_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

                        tu = int(self.filtered_u)
                        tv = int(self.filtered_v)
                        cv2.circle(display, (tu, tv), 6, (255, 0, 0), -1)
                        cv2.line(display, (cx_img, cy_img), (tu, tv), (255, 255, 0), 2)

                        debug_lines = [
                            f"LOCKED ID: {self.locked_id}",
                            f"u, v = ({self.filtered_u:.1f}, {self.filtered_v:.1f}) px",
                            f"bbox_w, bbox_h = ({bw}, {bh}) px",
                        ]

                        if self.camera_info_received:
                            fx_s, fy_s, cx_s, cy_s = self.get_scaled_intrinsics()
                            debug_lines.append(
                                f"fx={fx_s:.2f}, fy={fy_s:.2f}, cx={cx_s:.2f}, cy={cy_s:.2f}"
                            )
                        else:
                            debug_lines.append("CameraInfo: not received")

                        if z_cam is not None and x_cam is not None:
                            debug_lines.append(f"X = {x_cam:.3f} m")
                            debug_lines.append(f"Y = {y_cam:.3f} m")
                            debug_lines.append(f"Z = {z_cam:.3f} m")
                        else:
                            debug_lines.append("X, Y, Z = unavailable")

                        y0 = 30
                        for line in debug_lines:
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

                        break

        if self.locked_id is not None and not target_found:
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
                self.locked_id = None
                self.filtered_u = None
                self.filtered_v = None
                self.filtered_z = None
                self.has_valid_pose = False
                self.last_seen_time = None
                self.pending_select_bbox = None
                self.get_logger().info("Target reset")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLockTrackerNode()
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