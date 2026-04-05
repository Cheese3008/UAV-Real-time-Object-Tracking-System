#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, String


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_frame = None
        self.frame_width = 0
        self.frame_height = 0


class WebBridgeNode(Node):
    def __init__(self, shared_state: SharedState):
        super().__init__("web_bridge_node")
        self.shared_state = shared_state
        self.bridge = CvBridge()
        self.first_frame_logged = False

        self.declare_parameter("image_topic", "/image_proc")
        self.declare_parameter("click_point_topic", "/click_point")
        self.declare_parameter("select_bbox_topic", "/select_bbox")
        self.declare_parameter("reset_topic", "/follow_reset")

        image_topic = self.get_parameter("image_topic").value
        click_point_topic = self.get_parameter("click_point_topic").value
        select_bbox_topic = self.get_parameter("select_bbox_topic").value
        reset_topic = self.get_parameter("reset_topic").value

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.click_pub = self.create_publisher(
            PointStamped,
            click_point_topic,
            10
        )

        self.select_bbox_pub = self.create_publisher(
            Float32MultiArray,
            select_bbox_topic,
            10
        )

        self.reset_pub = self.create_publisher(
            String,
            reset_topic,
            10
        )

        self.get_logger().info(f"Subscribed image: {image_topic}")
        self.get_logger().info(f"Publishing click: {click_point_topic}")
        self.get_logger().info(f"Publishing select bbox: {select_bbox_topic}")
        self.get_logger().info(f"Publishing reset: {reset_topic}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.shared_state.lock:
                self.shared_state.latest_frame = frame
                self.shared_state.frame_height, self.shared_state.frame_width = frame.shape[:2]

            if not self.first_frame_logged:
                self.first_frame_logged = True
                self.get_logger().info(
                    f"First frame received: {frame.shape[1]}x{frame.shape[0]}"
                )
        except Exception as e:
            self.get_logger().error(f"image_callback error: {e}")

    def publish_click(self, x: float, y: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.click_pub.publish(msg)
        self.get_logger().info(f"Published click_point: ({x:.1f}, {y:.1f})")

    def publish_select_bbox(self, x: float, y: float, w: float, h: float):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(w), float(h)]
        self.select_bbox_pub.publish(msg)

    def publish_reset(self):
        msg = String()
        msg.data = "RESET"
        self.reset_pub.publish(msg)
        self.get_logger().info("Published RESET")


def _spin_ros(node: WebBridgeNode):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        node.get_logger().info("ROS spin stopped by external shutdown")
    except Exception as e:
        node.get_logger().error(f"ROS spin error: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def start_ros_node(shared_state: SharedState):
    if not rclpy.ok():
        rclpy.init()

    node = WebBridgeNode(shared_state)
    thread = threading.Thread(target=_spin_ros, args=(node,), daemon=True)
    thread.start()
    return node, thread