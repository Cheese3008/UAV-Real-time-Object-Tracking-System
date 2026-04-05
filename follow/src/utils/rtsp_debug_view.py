#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class ArucoProcViewer(Node):
    def __init__(self):
        super().__init__('aruco_proc_viewer')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/image_proc',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("ArUco processed image viewer started")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow("ArUco Processed Output", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")


def main():
    rclpy.init()
    node = ArucoProcViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
