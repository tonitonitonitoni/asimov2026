#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

import numpy as np
import cv2
from ultralytics import YOLO #TODO: add to the install list in the README
from message_filters import ApproximateTimeSynchronizer, Subscriber


class YoloSizeNode(Node):
    def __init__(self):
        super().__init__("yolov11_object_size_node")

        # YOLOv11 model
        self.model = YOLO("yolo11n.pt")

        # Time-synchronized subscribers
        rgb_sub = Subscriber(self, Image, "/depth_camera/image_raw")
        depth_sub = Subscriber(self, Image, "/depth_camera/depth/image_raw")
        info_sub = Subscriber(self, CameraInfo, "/depth_camera/camera_info")

        self.ts = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)

        # Publishers
        self.pub_det = self.create_publisher(Detection3DArray, "/detected_objects", 10)
        self.pub_debug = self.create_publisher(Image, "/debug_image", 10)

        self.get_logger().info("YOLOv11 + Depth Size Estimation Node Started")

    # -----------------------------------------------------------
    # Convert ROS2 Image message → BGR np.ndarray (your function)
    # -----------------------------------------------------------
    def image_msg_to_bgr(self, msg: Image):
        encoding = msg.encoding.lower()
        channels_map = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}
        dtype_map = {'bgr8': np.uint8, 'rgb8': np.uint8, 'mono8': np.uint8}

        channels = channels_map.get(encoding)
        dtype = dtype_map.get(encoding)

        if channels is None:
            self.get_logger().warn(f"Unsupported color encoding: {msg.encoding}")
            return None

        frame = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame

    # -----------------------------------------------------------
    # Convert depth image msg → float32 numpy array (meters)
    # Works for: 16UC1, 32FC1, 8UC1 (rare)
    # -----------------------------------------------------------
    def depth_msg_to_array(self, msg: Image):
        enc = msg.encoding.lower()

        if enc in ("16uc1", "mono16"):
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth = depth.astype(np.float32) / 1000.0  # convert mm → meters

        elif enc in ("32fc1",):
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
            return None

        return depth

    # -----------------------------------------------------------
    # Main callback
    # -----------------------------------------------------------
    def callback(self, rgb_msg, depth_msg, info_msg):
        rgb = self.image_msg_to_bgr(rgb_msg)
        depth = self.depth_msg_to_array(depth_msg)
        im_h, im_w, _ = rgb.shape
        if rgb is None or depth is None:
            return

        # Camera intrinsics
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        # YOLOv11 inference
        results = self.model(rgb, verbose=False)

        detections_msg = Detection3DArray()
        detections_msg.header = rgb_msg.header

        debug_img = rgb.copy()

        for i, det in enumerate(results[0].boxes):
            x1, y1, x2, y2 = map(int, det.xyxy[0].cpu().numpy())
            cls_id = int(det.cls.cpu().numpy()[0])
            conf = float(det.conf.cpu().numpy()[0])

            # Depth crop for median filtering
            crop = depth[y1:y2, x1:x2]
            valid = crop[crop > 0.1]  # ignore zeros / invalid

            if len(valid) == 0:
                continue

            Z = float(np.median(valid))  # meters

            # Pixel size
            w_px = x2 - x1
            h_px = y2 - y1

            # Convert pixel → metric using pinhole camera model
            W = Z * w_px / fx
            H = Z * h_px / fy


            # Debug drawing
            if i==0:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2
                colour = (0,255,0)
                message = f"{self.model.names[cls_id]} {W:.2f}m x {H:.2f}m"
                (text_width, text_height), baseline = cv2.getTextSize(message, font, font_scale, thickness)
                
                y_t = y1 - 15 
                if y_t < text_height+10:
                    y_t = y2 + 15
                
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), colour, thickness)
                cv2.putText(
                    debug_img,
                    message,
                    (x1, y_t),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    colour,
                    thickness
                )
            

            # Create ROS2 Detection3D message
            det3d = Detection3D()
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[cls_id]   # must be a string
            hypothesis.hypothesis.score = float(conf)                   # confidence

            det3d.results.append(hypothesis)

            # Object center pixel
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0

            # Back-project to 3D camera coordinates
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            det3d.bbox.center.position.x = float(X)
            det3d.bbox.center.position.y = float(Y)
            det3d.bbox.center.position.z = float(Z)
            det3d.bbox.size.x = float(W)
            det3d.bbox.size.y = float(H)
            det3d.bbox.size.z = 0.05  # thin-box approximation

            detections_msg.detections.append(det3d)
        cv2.imshow("Resultant Image", debug_img)
        cv2.waitKey(1)
        # Publish results
        self.pub_det.publish(detections_msg)

        # Publish debug image (we must convert np.array → raw ROS2 Image)
        debug_msg = self.ndarray_to_image_msg(debug_img, rgb_msg)
        self.pub_debug.publish(debug_msg)

    # -----------------------------------------------------------
    # Convert numpy BGR → ROS2 Image message
    # -----------------------------------------------------------
    def ndarray_to_image_msg(self, img, ref_msg):
        msg = Image()
        msg.header = ref_msg.header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloSizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
