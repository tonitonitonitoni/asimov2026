#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
import time


class PanoramaSweeper(Node):

    def __init__(self):
        super().__init__('panorama_sweeper')

        # Image + angle buffers
        self.frames = []
        self.angles = []

        # Joint state
        self.current_angle = None

        # Sweep config (in degrees)
        self.sweep_angles = np.linspace(-80, 80, 13)  # 13 steps, 15° apart
        self.sweep_index = 0
        self.angle_tolerance_deg = 2.0           # must reach within 2°
        self.has_image_since_reaching_angle = False

        # For simple state machine
        self.state = "IDLE"   # IDLE → SWEEPING → STITCHING → DONE

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)

        # Publisher: position command for servo
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/camera_pan_position_controller/commands',
            10
        )


        # Kick off sweep after a short delay
        self.create_timer(2.0, self.start_sweep)

        self.get_logger().info("PanoramaSweeper node started")

    # ---------------------------------------------------------
    # Start sweep
    # ---------------------------------------------------------
    def start_sweep(self):
        if self.state == "IDLE":
            self.get_logger().info("Starting camera sweep...")
            self.state = "SWEEPING"
            self.move_to_next_angle()

    # ---------------------------------------------------------
    # Command servo to next sweep angle
    # ---------------------------------------------------------
    def move_to_next_angle(self):
        if self.sweep_index >= len(self.sweep_angles):
            self.get_logger().info("Sweep complete → Stitching panorama")
            self.state = "STITCHING"
            self.stitch_panorama()
            return

        target_deg = self.sweep_angles[self.sweep_index]
        target_rad = math.radians(target_deg)

        msg = Float64MultiArray()
        msg.data = [float(target_rad)]
        self.cmd_pub.publish(msg)

        self.get_logger().info(f"Moving to {target_deg:.1f}°")
        #TODO: Feedback is not working
        self.has_image_since_reaching_angle = False
    
    # ---------------------------------------------------------
    # Joint callback
    # ---------------------------------------------------------
    def joint_callback(self, msg: JointState):
        if "camera_pan_joint" in msg.name:
            idx = msg.name.index("camera_pan_joint")
            self.current_angle = msg.position[idx]

    # ---------------------------------------------------------
    # Image callback
    # ---------------------------------------------------------
    def image_callback(self, msg: Image):
        if self.state != "SWEEPING":
            return

        if self.current_angle is None:
            return

        frame = self.image_msg_to_bgr(msg)
        if frame is None:
            return

        # Check if we've reached the target angle
        target_deg = self.sweep_angles[self.sweep_index]
        
        current_deg = math.degrees(self.current_angle)
        print(f"{current_deg:.2f}")

        if abs(current_deg - target_deg) < self.angle_tolerance_deg:
            if not self.has_image_since_reaching_angle:
                # Capture image only once per sweep point
                self.frames.append(frame)
                self.angles.append(current_deg)
                self.has_image_since_reaching_angle = True

                self.get_logger().info(
                    f"Captured frame at {current_deg:.1f}° "
                    f"({len(self.frames)}/{len(self.sweep_angles)})"
                )

                # Move to next sweep angle
                self.sweep_index += 1
                # Slight delay to avoid command overlap
                time.sleep(0.3)
                self.move_to_next_angle()

    # ---------------------------------------------------------
    # Image msg to BGR (NO CVBRIDGE)
    # ---------------------------------------------------------
    def image_msg_to_bgr(self, msg: Image):
        encoding = msg.encoding.lower()
        channels_map = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}
        dtype_map = {'bgr8': np.uint8, 'rgb8': np.uint8, 'mono8': np.uint8}

        channels = channels_map.get(encoding)
        dtype = dtype_map.get(encoding)

        if channels is None:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return None

        frame = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, msg.width, channels
        )

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame

    # ---------------------------------------------------------
    # Stitch images into panorama
    # ---------------------------------------------------------
    def stitch_panorama(self):
        if len(self.frames) < 2:
            self.get_logger().warn("Not enough frames to stitch")
            return

        self.get_logger().info(f"Stitching {len(self.frames)} images...")

        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, pano = stitcher.stitch(self.frames)

        if status != cv2.Stitcher_OK:
            self.get_logger().error(f"Stitch failed with status {status}")
            return

        cv2.imwrite("panorama.jpg", pano)
        self.get_logger().info("Panorama saved → panorama.jpg")

        cv2.imshow("Panorama", pano)
        cv2.waitKey(1)

        self.state = "DONE"


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
