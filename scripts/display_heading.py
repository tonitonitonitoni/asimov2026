#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def yaw_from_quaternion(q) -> float:
    """Compute yaw (heading) in radians from geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def heading_to_compass(degrees: float) -> str:
    """Convert heading in degrees (clockwise from north) to compass text."""
    if degrees is None:
        return '--'

    angle = round(degrees) % 360

    if angle == 0: return 'N'
    if angle == 90: return 'E'
    if angle == 180: return 'S'
    if angle == 270: return 'W'
    if angle < 90: return f'N{angle}E'
    if angle < 180: return f'S{180-angle}E'
    if angle < 270: return f'S{angle-180}W'
    return f'N{360-angle}W'


class HeadingOverlayNode(Node):
    def __init__(self):
        super().__init__('heading_overlay')

        self.heading_deg = None

        # Use EKF output instead of IMU
        self.create_subscription(
            Odometry,
            '/odometry/global',
            self.ekf_callback,
            10
        )

        self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10
        )
        self.pub_overlay = self.create_publisher(Image, "/overlay_image", 10)

    # Get heading from EKF
    def ekf_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q)

        # Convert to compass heading (North=0°, East=90°)
        self.heading_deg = (math.degrees(yaw) + 360.0) % 360.0

    def add_heading(self, frame):
        H, W, _ = frame.shape
        heading_text = heading_to_compass(self.heading_deg)
        label = f'Heading: {heading_text}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        pad = 10
        x_c = (W - text_width)//2
        y_c = pad + text_height
        colour = (0, 0, 255)
        
        cv2.putText(
            frame,
            label,
            (x_c, y_c),
            font,
            font_scale,
            colour,
            thickness,
            cv2.LINE_AA,
        )
    def image_callback(self, msg: Image):
        frame = self.image_msg_to_bgr(msg)
        if frame is None:
            return

        self.add_heading(frame)

        cv2.imshow('Heading Overlay', frame)
        cv2.waitKey(1)
        
        self.pub_overlay.publish(self.ndarray_to_image_msg(frame, msg))
    
    def ndarray_to_image_msg(self, img, ref):
        msg = Image()
        msg.header = ref.header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg


    def image_msg_to_bgr(self, msg: Image):
        encoding = msg.encoding.lower()
        channels_map = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}
        dtype_map = {'bgr8': np.uint8, 'rgb8': np.uint8, 'mono8': np.uint8}

        channels = channels_map.get(encoding)
        dtype = dtype_map.get(encoding)

        if channels is None or dtype is None:
            self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")
            return None

        frame = np.frombuffer(msg.data, dtype).reshape(msg.height, msg.width, channels)

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = HeadingOverlayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
