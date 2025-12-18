#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
import numpy as np
import cv2
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber


class YoloDepthAccurate(Node):
    def __init__(self):
        super().__init__("yolo_depth_accurate")

        # Load YOLO
        self.model = YOLO("yolo11n.pt")

        # Subscribers
        rgb_sub = Subscriber(self, Image, "/depth_camera/image_raw")
        depth_sub = Subscriber(self, Image, "/depth_camera/depth/image_raw")
        info_sub = Subscriber(self, CameraInfo, "/depth_camera/depth/camera_info")

        self.ts = ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub],
                                              queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # Publishers
        self.pub_det = self.create_publisher(Detection3DArray, "/detected_objects", 10)
        self.pub_debug = self.create_publisher(Image, "/debug_image", 10)

        self.get_logger().info("High-Accuracy YOLO + Depth Estimation Node Started")

    # -------------------------------
    # Convert color image
    # -------------------------------
    def image_msg_to_bgr(self, msg: Image):
        encoding = msg.encoding.lower()
        channels_map = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}
        dtype_map = {'bgr8': np.uint8, 'rgb8': np.uint8, 'mono8': np.uint8}

        ch = channels_map.get(encoding)
        dt = dtype_map.get(encoding)
        if ch is None:
            self.get_logger().warn(f"Unsupported color encoding: {msg.encoding}")
            return None

        frame = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.width, ch)

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame

    # -------------------------------
    # Convert depth image msg
    # -------------------------------
    def depth_msg_to_array(self, msg: Image):
        enc = msg.encoding.lower()

        if enc in ("16uc1", "mono16"):
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth = depth.astype(np.float32) / 1000.0  # mm → meters
        elif enc == "32fc1":
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
            return None

        # Apply small median blur to reduce pixel noise
        depth = cv2.medianBlur(depth, 3)

        return depth

    # -------------------------------
    # Convert np.array → ROS2 Image
    # -------------------------------
    def ndarray_to_image_msg(self, img, ref_msg):
        msg = Image()
        msg.header = ref_msg.header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg

    # -------------------------------
    # MAIN CALLBACK
    # -------------------------------
    def callback(self, rgb_msg, depth_msg, info_msg):
        rgb = self.image_msg_to_bgr(rgb_msg)
        depth = self.depth_msg_to_array(depth_msg)
        if rgb is None or depth is None:
            return

        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        results = self.model(rgb, verbose=False)

        det_array = Detection3DArray()
        det_array.header = rgb_msg.header

        debug_img = rgb.copy()

        for det in results[0].boxes:
            x1, y1, x2, y2 = map(int, det.xyxy[0].cpu().numpy())
            cls_id = int(det.cls.cpu().numpy())
            conf = float(det.conf.cpu().numpy())
            label = self.model.names[cls_id]

            # Clamp to image bounds just in case
            h_img, w_img = depth.shape
            x1 = max(0, min(x1, w_img - 1))
            x2 = max(0, min(x2, w_img - 1))
            y1 = max(0, min(y1, h_img - 1))
            y2 = max(0, min(y2, h_img - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            w = x2 - x1
            h = y2 - y1

            # ----------------------------------------------------
            # 1) Get a robust *central* depth estimate
            #    (use inner 40% of the box to avoid edges/background)
            # ----------------------------------------------------
            edge_remove = 0.1
            cx1 = int(x1 + edge_remove * w)
            cx2 = int(x2 - edge_remove * w)
            cy1 = int(y1 + edge_remove * h)
            cy2 = int(y2 - edge_remove * h)

            if cx2 <= cx1 or cy2 <= cy1:
                continue

            center_crop = depth[cy1:cy2, cx1:cx2]
            valid_center = center_crop[(center_crop > 0.1) & (center_crop < 20.0)]

            if len(valid_center) == 0:
                # fallback: whole box
                crop_full = depth[y1:y2, x1:x2]
                valid_full = crop_full[(crop_full > 0.1) & (crop_full < 20.0)]
                if len(valid_full) == 0:
                    continue
                Z = float(np.median(valid_full))
            else:
                Z = float(np.median(valid_center))

            # ----------------------------------------------------
            # 2) Use *YOLO box* width/height in pixels
            #    (silhouette mask was making things worse)
            # ----------------------------------------------------
            w_px = w
            h_px = h

            # Metric size using pinhole model
            W = (Z * w_px) / fx
            H = (Z * h_px) / fy

            # Debug draw
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug_img,
                        f"{label} {W:.2f}m x {H:.2f}m",
                        (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)

            # ----------------------------------------------------
            # 3) Fill Detection3D
            # ----------------------------------------------------
            det3d = Detection3D()

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label
            hypo.hypothesis.score = conf
            det3d.results.append(hypo)

            u = 0.5 * (x1 + x2)
            v = 0.5 * (y1 + y2)

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            det3d.bbox.center.position.x = float(X)
            det3d.bbox.center.position.y = float(Y)
            det3d.bbox.center.position.z = float(Z)

            det3d.bbox.size.x = float(W)
            det3d.bbox.size.y = float(H)
            det3d.bbox.size.z = 0.05  # arbitrary thin depth

            det_array.detections.append(det3d)

            # Optional debug logging
            self.get_logger().debug(
                f"{label}: Z={Z:.3f} m, w_px={w_px}, h_px={h_px}, "
                f"W={W:.3f} m, H={H:.3f} m"
            )
        cv2.imshow("Resultant Image", debug_img)
        cv2.waitKey(1)
        self.pub_det.publish(det_array)
        debug_msg = self.ndarray_to_image_msg(debug_img, rgb_msg)
        self.pub_debug.publish(debug_msg)



def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthAccurate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
