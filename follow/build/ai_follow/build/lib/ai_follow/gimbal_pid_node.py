#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String


class GimbalPIDNode(Node):
    def __init__(self):
        super().__init__("gimbal_pid_node")

        # ================= PARAMETERS =================
        self.declare_parameter("target_pose_topic", "/detecd_pose")
        self.declare_parameter("cmd_speed_topic", "/gimbal/cmd/speed")
        self.declare_parameter("tag_state_topic", "/tag_state")

        self.declare_parameter("control_rate", 20.0)

        # PID yaw (điều khiển theo x)
        self.declare_parameter("kp_yaw", 12.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.0)

        # PID pitch (điều khiển theo y)
        self.declare_parameter("kp_pitch", 12.0)
        self.declare_parameter("ki_pitch", 0.0)
        self.declare_parameter("kd_pitch", 0.0)

        self.declare_parameter("max_yaw_speed", 10.0)
        self.declare_parameter("max_pitch_speed", 10.0)

        self.declare_parameter("deadband_x", 0.03)
        self.declare_parameter("deadband_y", 0.03)

        self.declare_parameter("integral_limit_yaw", 0.5)
        self.declare_parameter("integral_limit_pitch", 0.5)

        # nếu mất target thì dừng gimbal
        self.declare_parameter("stop_on_lost", True)

        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.cmd_speed_topic = self.get_parameter("cmd_speed_topic").value
        self.tag_state_topic = self.get_parameter("tag_state_topic").value
        self.control_rate = float(self.get_parameter("control_rate").value)

        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.ki_yaw = float(self.get_parameter("ki_yaw").value)
        self.kd_yaw = float(self.get_parameter("kd_yaw").value)

        self.kp_pitch = float(self.get_parameter("kp_pitch").value)
        self.ki_pitch = float(self.get_parameter("ki_pitch").value)
        self.kd_pitch = float(self.get_parameter("kd_pitch").value)

        self.max_yaw_speed = float(self.get_parameter("max_yaw_speed").value)
        self.max_pitch_speed = float(self.get_parameter("max_pitch_speed").value)

        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)

        self.integral_limit_yaw = float(self.get_parameter("integral_limit_yaw").value)
        self.integral_limit_pitch = float(self.get_parameter("integral_limit_pitch").value)

        self.stop_on_lost = bool(self.get_parameter("stop_on_lost").value)

        # ================= QoS =================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ================= STATE =================
        self.target_x = None
        self.target_y = None
        self.has_target = False

        self.last_time = time.time()

        self.int_yaw = 0.0
        self.int_pitch = 0.0

        self.prev_err_yaw = 0.0
        self.prev_err_pitch = 0.0

        # ================= ROS I/O =================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_pose_cb,
            qos
        )

        self.tag_state_sub = self.create_subscription(
            String,
            self.tag_state_topic,
            self.tag_state_cb,
            qos
        )

        self.cmd_pub = self.create_publisher(
            Vector3,
            self.cmd_speed_topic,
            qos
        )

        period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("Gimbal PID node started")
        self.get_logger().info(f"Sub target pose: {self.target_pose_topic}")
        self.get_logger().info(f"Pub gimbal speed: {self.cmd_speed_topic}")

    # ==================================================
    # CALLBACKS
    # ==================================================
    def target_pose_cb(self, msg: PoseStamped):
        self.target_x = float(msg.pose.position.x)
        self.target_y = float(msg.pose.position.y)
        self.has_target = True

    def tag_state_cb(self, msg: String):
        self.has_target = (msg.data == "1")

    # ==================================================
    # HELPERS
    # ==================================================
    @staticmethod
    def clamp(val: float, low: float, high: float) -> float:
        return max(low, min(high, val))

    def reset_pid(self):
        self.int_yaw = 0.0
        self.int_pitch = 0.0
        self.prev_err_yaw = 0.0
        self.prev_err_pitch = 0.0

    def publish_speed(self, yaw_speed: float, pitch_speed: float):
        cmd = Vector3()
        cmd.x = float(yaw_speed)   # driver của bạn đang dùng msg.x cho yaw
        cmd.y = float(pitch_speed) # msg.y cho pitch
        cmd.z = 0.0
        self.cmd_pub.publish(cmd)

    # ==================================================
    # CONTROL LOOP
    # ==================================================
    def control_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 1e-4:
            return

        if not self.has_target or self.target_x is None or self.target_y is None:
            self.reset_pid()
            if self.stop_on_lost:
                self.publish_speed(0.0, 0.0)
            return

        # Mục tiêu: x -> 0, y -> 0
        # err_yaw điều khiển từ x
        # err_pitch điều khiển từ y
        err_yaw = self.target_x
        err_pitch = self.target_y

        # deadband
        if abs(err_yaw) < self.deadband_x:
            err_yaw = 0.0
        if abs(err_pitch) < self.deadband_y:
            err_pitch = 0.0

        # integral
        self.int_yaw += err_yaw * dt
        self.int_pitch += err_pitch * dt

        self.int_yaw = self.clamp(
            self.int_yaw, -self.integral_limit_yaw, self.integral_limit_yaw
        )
        self.int_pitch = self.clamp(
            self.int_pitch, -self.integral_limit_pitch, self.integral_limit_pitch
        )

        # derivative
        der_yaw = (err_yaw - self.prev_err_yaw) / dt
        der_pitch = (err_pitch - self.prev_err_pitch) / dt

        self.prev_err_yaw = err_yaw
        self.prev_err_pitch = err_pitch

        # PID
        yaw_cmd = (
            self.kp_yaw * err_yaw +
            self.ki_yaw * self.int_yaw +
            self.kd_yaw * der_yaw
        )

        pitch_cmd = - (
            self.kp_pitch * err_pitch +
            self.ki_pitch * self.int_pitch +
            self.kd_pitch * der_pitch
        )

        # Có thể cần đảo dấu 1 trong 2 trục tùy gimbal thực tế
        # Nếu quay sai chiều, đổi dấu ở đây:
        yaw_cmd = self.clamp(yaw_cmd, -self.max_yaw_speed, self.max_yaw_speed)
        pitch_cmd = self.clamp(pitch_cmd, -self.max_pitch_speed, self.max_pitch_speed)

        self.publish_speed(yaw_cmd, pitch_cmd)

        self.get_logger().info(
            f"x={self.target_x:.3f}, y={self.target_y:.3f} | "
            f"yaw_cmd={yaw_cmd:.1f}, pitch_cmd={pitch_cmd:.1f}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GimbalPIDNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()