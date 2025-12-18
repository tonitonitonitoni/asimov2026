#!/usr/bin/env python3
import math
import cv2, os, re
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
import time
from sensor_msgs.msg import Image, Imu


def yaw_from_quaternion(q) -> float:
    """Compute yaw (heading) in radians from geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def heading_to_compass(degrees: float) -> str:
    """Convert heading in degrees (clockwise from north) to a compass string."""
    if degrees is None:
        return '--'

    angle = round(degrees) % 360
    if angle == 0:
        return 'N'
    if angle == 90:
        return 'E'
    if angle == 180:
        return 'S'
    if angle == 270:
        return 'W'
    if angle < 90:
        return f'N{angle}E'
    if angle < 180:
        return f'S{180 - angle}E'
    if angle < 270:
        return f'S{angle - 180}W'
    return f'N{360 - angle}W'

def next_filename(stem, folder="panoramas", suffix=".jpg"):
    """
    Return the next available filename with numeric suffix.
    Example: prefix='good_poses' -> 'good_poses_0.pkl' -> 'good_poses_1.pkl', etc.
    """

    pattern = re.compile(rf"^{re.escape(stem)}(\d+){re.escape(suffix)}$")
    max_num = -1

    for fname in os.listdir(folder):
        match = pattern.match(fname)
        if match:
            num = int(match.group(1))
            if num > max_num:
                max_num = num

    next_num = max_num + 1
    if folder is not None:
        return f"{folder}/{stem}{next_num}{suffix}"
    else:
        return f"{stem}{next_num}{suffix}"
    
class StitcherWithOverlay(Node):

    def __init__(self):
        super().__init__('overlay_stitcher')

        # Image + angle buffers
        self.frames = []
        self.angles = []

        # Joint state
        self.current_angle = None

        # Sweep config (in degrees)
        self.n_steps = 5
        self.step_size = 15
        self.max_angle = self.n_steps*self.step_size
        self.min_angle = -self.max_angle
        self.sweep_angles = np.linspace(self.min_angle, self.max_angle, 2*self.n_steps+1, endpoint=True)  # 13 steps, 15° apart
        self.sweep_index = 0
        self.angle_tolerance_deg = 2.0           # must reach within 2°
        self.has_image_since_reaching_angle = False

        # For simple state machine
        self.state = "IDLE"   # IDLE → SWEEPING → STITCHING → DONE

        # Subscribers
        self.heading_deg = None

        self.create_subscription(Imu, '/demo/imu', self.imu_callback, 10)
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
            target_deg = 0
            target_rad = math.radians(target_deg)
            msg = Float64MultiArray()
            msg.data = [float(target_rad)]
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Moving to {target_deg:.1f}°")
            return

        target_deg = self.sweep_angles[self.sweep_index]
        target_rad = math.radians(target_deg)

        msg = Float64MultiArray()
        msg.data = [float(target_rad)]
        self.cmd_pub.publish(msg)

        self.get_logger().info(f"Moving to {target_deg:.1f}°")
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
    def add_heading(self, frame):
        H, W, _ = frame.shape
        heading_text = heading_to_compass(self.heading_deg)
        label = f'Heading: {heading_text}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        (text_width, text_height), _ = cv2.getTextSize(label, font, font_scale, thickness)
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
    # IMU callback (later will be odometry/filtered)
    # ---------------------------------------------------------
    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = yaw_from_quaternion(q)
        self.heading_deg = (math.degrees(yaw) + 360.0) % 360.0


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
    def _stitch_once(self, frames, mode, confidence_thresh=0.6):
        stitcher = cv2.Stitcher_create(mode)
        # if confidence_thresh is not None:
        #     try:
        #         confidence = 1.0 if (not math.isfinite(confidence_thresh) or confidence_thresh <= 0.0) else confidence_thresh
        #         stitcher.setPanoConfidenceThresh(confidence)
        #     except cv2.error:
        #         pass
        return stitcher.stitch(frames)

    def stitch_panorama(self):
        if len(self.frames) < 2:
            self.get_logger().warn("Not enough frames to stitch")
            return

        self.get_logger().info(f"Stitching {len(self.frames)} images...")

        frames = [frame for _, frame in sorted(zip(self.angles, self.frames))]

        status, pano = self._stitch_once(frames, cv2.Stitcher_PANORAMA, 0.45)

        if status == cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL:
            self.get_logger().warn("Stitch failed (camera params). Retrying in SCANS mode")
            status, pano = self._stitch_once(frames, cv2.Stitcher_SCANS, 0.3)

        if status != cv2.Stitcher_OK:
            self.get_logger().error(f"Stitch failed with status {status}")
            return

        self.add_heading(pano) 
        fname = next_filename("panorama")
        cv2.imwrite(fname, pano)
        self.get_logger().info(f"Panorama saved → {fname}")

        cv2.imshow("Panorama", pano)
        cv2.waitKey(1)

        self.state = "DONE"


def main(args=None):
    rclpy.init(args=args)
    node = StitcherWithOverlay()
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
