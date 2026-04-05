#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .siyi_sdk.siyi_sdk import SIYISDK

import threading
import time


class SiyiGimbalNode(Node):
    def __init__(self):
        super().__init__('siyi_gimbal_node')

        # ================= PARAMETERS =================
        self.declare_parameter('ip', '192.168.144.25')
        self.declare_parameter('port', 37260)
        self.declare_parameter('state_rate', 20.0)  # Hz

        ip = self.get_parameter('ip').value
        port = self.get_parameter('port').value
        rate = self.get_parameter('state_rate').value

        # ================= INTERNAL STATE =================
        self._connected = False
        self._running = True

        # ================= SIYI SDK =================
        self.gimbal = SIYISDK(server_ip=ip, port=port, debug=False)

        self.get_logger().info('Connecting to SIYI gimbal (UDP control only)...')
        if not self.gimbal.connect():
            raise RuntimeError('Failed to connect to SIYI gimbal')

        self._connected = True
        self.get_logger().info(
            f'Connected to SIYI gimbal: {self.gimbal.getCameraTypeString()}'
        )
        # ================= QoS =================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # ================= SUBSCRIBERS =================
        self.create_subscription(
            Vector3,
            '/gimbal/cmd/angle',
            self.cmd_angle_cb,
            qos_profile
        )

        self.create_subscription(
            Vector3,
            '/gimbal/cmd/speed',
            self.cmd_speed_cb,
            qos_profile
        )

        self.create_subscription(
            String,
            '/gimbal/cmd/mode',
            self.cmd_mode_cb,
            qos_profile
        )

        self.create_subscription(
            String,
            '/gimbal/cmd/sequence',
            self.cmd_sequence_cb,
            qos_profile
        )

        # ================= PUBLISHERS =================
        self.att_pub = self.create_publisher(
            Vector3,
            '/gimbal/state/attitude',
            qos_profile
        )

        self.zoom_pub = self.create_publisher(
            Float32,
            '/gimbal/state/zoom',
            qos_profile
        )

        # ================= STATE LOOP =================
        period = 1.0 / max(rate, 1.0)
        self.create_timer(period, self.publish_state)

        self.get_logger().info('SIYI Gimbal UDP Driver READY')

    # ==================================================
    #                    CALLBACKS
    # ==================================================

    def cmd_angle_cb(self, msg: Vector3):
        if not self._connected:
            return
        self.gimbal.requestSetAngles(msg.x, msg.y)

    def cmd_speed_cb(self, msg: Vector3):
        if not self._connected:
            return
        self.gimbal.requestGimbalSpeed(int(msg.x), int(msg.y))

    def cmd_mode_cb(self, msg: String):
        if not self._connected:
            return

        mode = msg.data.upper()
        if mode == 'FPV':
            self.gimbal.requestFPVMode()
        elif mode == 'FOLLOW':
            self.gimbal.requestFollowMode()
        elif mode == 'LOCK':
            self.gimbal.requestLockMode()
        else:
            self.get_logger().warn(f'Unknown gimbal mode: {mode}')

    def cmd_sequence_cb(self, msg: String):
        if not self._connected:
            return
            
        if msg.data == 'CENTER_LOOKDOWN_FPV':
            threading.Thread(
                target=self.seq_center_lookdown_fpv,
                daemon=True
            ).start()
        elif msg.data == 'CENTER_LOOKUP_FOLLOW':
            threading.Thread(
                target=self.seq_center_lookup_follow,
                daemon=True
            ).start()
        elif msg.data == 'CENTER_LOOKDOWN_FOLLOW':
            threading.Thread(
                target=self.seq_center_lookdown_follow,
                daemon=True
            ).start()
        elif msg.data == 'CENTER_LOOKDOWN_LOCK':
            threading.Thread(
                target=self.seq_center_lookdown_lock,
                daemon=True
            ).start()
        else:
            self.get_logger().warn(f'Unknown gimbal sequence: {msg.data}')

    # ==================================================
    #                    SEQUENCES
    # ==================================================

    def seq_center_lookdown_fpv(self):
        self.gimbal.requestFPVMode()
        time.sleep(0.3)
        self.gimbal.requestSetAngles(0.0, -90.0)

    def seq_center_lookup_follow(self):
        self.get_logger().info('SEQ: Lookup + FOLLOW')
        self.gimbal.requestFollowMode()
        time.sleep(0.3)
        self.gimbal.requestSetAngles(0.0, 50.0)
        
    def seq_center_lookdown_follow(self):
        self.get_logger().info('SEQ: Lookdown + FOLLOW')
        self.gimbal.requestFollowMode()
        time.sleep(0.3)
        self.gimbal.requestSetAngles(0.0, -90.0)

    def seq_center_lookdown_lock(self):
        self.get_logger().info('SEQ: Lookdown + LOCK')
        self.gimbal.requestLockMode()
        time.sleep(0.3)
        self.gimbal.requestSetAngles(0.0, -90.0)


    # ==================================================
    #                STATE PUBLISH LOOP
    # ==================================================
    def publish_state(self):
        # Tất cả code bên dưới PHẢI thụt vào 1 mức (4 dấu cách)
        if not self._connected or not rclpy.ok():
            return

        try:
            # Thêm timeout hoặc kiểm tra nhanh ở đây nếu SDK hỗ trợ
            yaw, pitch, roll = self.gimbal.getAttitude()
            zoom = self.gimbal.getCurrentZoomLevel()
            
            # Nếu SDK trả về None hoặc giá trị lỗi, đừng publish
            if yaw is None:
                return

            att = Vector3()
            att.x = float(yaw)
            att.y = float(pitch)
            att.z = float(roll)
            self.att_pub.publish(att)

            z = Float32()
            z.data = float(zoom)
            self.zoom_pub.publish(z)

        except Exception as e:
            self.get_logger().error(f"Error in publish_state: {e}")

    # ==================================================
    #                CLEAN SHUTDOWN
    # ==================================================

    def destroy_node(self):
        self.get_logger().info('Stopping SIYI gimbal UDP driver')
        self._connected = False
        self._running = False
        try:
            self.gimbal.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SiyiGimbalNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node crashed: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


