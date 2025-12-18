#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
import numpy as np
import cv2
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber


class YoloDepthPointCloud(Node):
    def __init__(self):
        super().__init__("yolo_depth_pointcloud")

        # Load YOLO
        self.model = YOLO("yolo11n.pt")

        # Time-synchronized subscribers
        rgb_sub = Subscriber(self, Image, "/depth_camera/image_raw")
        depth_sub = Subscriber(self, Image, "/depth_camera/depth/image_raw")
        info_sub = Subscriber(self, CameraInfo, "/depth_camera/camera_info")

        self.ts = ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub],
                                              queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # Publishers
        self.pub_det = self.create_publisher(Detection3DArray, "/detected_objects", 10)
        self.pub_debug = self.create_publisher(Image, "/debug_image", 10)

        self.get_logger().info("YOLO + Depth Point Cloud Size Estimation Node Started")

    # --------------------------------------------------------------------------
    # Image converters (no cv_bridge)
    # --------------------------------------------------------------------------
    def image_msg_to_bgr(self, msg: Image):
        encoding = msg.encoding.lower()
        channels_map = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}
        dtype_map    = {'bgr8': np.uint8, 'rgb8': np.uint8, 'mono8': np.uint8}

        ch = channels_map.get(encoding)
        dt = dtype_map.get(encoding)

        if ch is None:
            self.get_logger().warn(f"Unsupported color encoding: {msg.encoding}")
            return None

        frame = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.width, ch)

        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == "mono8":
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame

    def depth_msg_to_array(self, msg: Image):
        enc = msg.encoding.lower()

        if enc in ("16uc1", "mono16"):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            arr = arr.astype(np.float32) / 1000.0  # mm → m
        elif enc == "32fc1":
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
            return None

        # Median filter to reduce noise
        return cv2.medianBlur(arr, 3)

    def ndarray_to_image_msg(self, img, ref):
        msg = Image()
        msg.header = ref.header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg

    # --------------------------------------------------------------------------
    # Auto-shape classification: sphere-like vs box-like
    # --------------------------------------------------------------------------
    def classify_shape(self, pts):
        # Compute distances from centroid
        center = pts.mean(axis=0)
        d = np.linalg.norm(pts - center, axis=1)

        # Sphere: low variance of radial distances
        var_ratio = np.std(d) / np.mean(d)

        # < 0.25 is very round
        # 0.25 - 0.40 somewhat round
        # > 0.40 not round (box-like / irregular)
        if var_ratio < 0.30:
            return "round", center, np.median(d)
        return "box", center, np.median(d)

    # --------------------------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------------------------
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
        if results is None: return
        det_array = Detection3DArray()
        det_array.header = rgb_msg.header

        debug_img = rgb.copy()

        for det in results[0].boxes:#[0]:
            x1, y1, x2, y2 = map(int, det.xyxy[0].cpu().numpy())
            cls_id = int(det.cls.cpu().numpy()[0])
            conf = float(det.conf.cpu().numpy()[0])
            label = self.model.names[cls_id]

            # Clamp to valid image bounds
            H, W = depth.shape
            x1 = max(0, min(x1, W - 1))
            x2 = max(0, min(x2, W - 1))
            y1 = max(0, min(y1, H - 1))
            y2 = max(0, min(y2, H - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            # ----------------------------------------------------------------------
            # 1) Extract point cloud from depth region
            # ----------------------------------------------------------------------
            ys, xs = np.mgrid[y1:y2, x1:x2]
            Z = depth[y1:y2, x1:x2]
            mask = (Z > 0.1) & (Z < 20.0)

            xs = xs[mask]
            ys = ys[mask]
            Z  = Z[mask]

            if len(Z) < 60:
                continue

            X = (xs - cx) * Z / fx
            Y = (ys - cy) * Z / fy
            pts = np.column_stack((X, Y, Z))

            # ----------------------------------------------------------------------
            # 2) Background removal: keep pts near median depth
            # ----------------------------------------------------------------------
            Z_med = np.median(pts[:, 2])
            pts = pts[np.abs(pts[:, 2] - Z_med) < 0.20]   # ±20 cm band

            if len(pts) < 50:
                continue

            # ----------------------------------------------------------------------
            # 3) Spatial outlier filtering
            # ----------------------------------------------------------------------
            center_approx = np.median(pts, axis=0)
            pts = pts[np.all(np.abs(pts - center_approx) < 0.8, axis=1)]
            if len(pts) < 50:
                continue

            # ----------------------------------------------------------------------
            # 4) Auto-shape classifier
            # ----------------------------------------------------------------------
            shape, center, median_radius = self.classify_shape(pts)

            if shape == "round":
                # Sphere diameter
                diameter = 2 * median_radius
                size_text = f"{diameter:.2f} m (sphere)"
                obj_W = obj_H = diameter

            else:
                # 3D bounding box
                min_xyz = pts.min(axis=0)
                max_xyz = pts.max(axis=0)
                dims = max_xyz - min_xyz   # [Wx, Hy, Dz]

                obj_W = float(dims[0])
                obj_H = float(dims[1])
                obj_D = float(dims[2])

                size_text = f"{obj_W:.2f} by {obj_H:.2f} by {obj_D:.2f} m"

            # ----------------------------------------------------------------------
            # 5) Draw debug box and text
            # ----------------------------------------------------------------------
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            colour = (0,255,0)
            (text_width, text_height), baseline = cv2.getTextSize(f"{label}: {size_text}", font, font_scale, thickness)
            x_c = (W - text_width)//2
            #y_t = (H + text_height)//2
            y_t = y1 - 15 
            if y_t < text_height+10:
                y_t = y2 + 15
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), colour, thickness)
            cv2.putText(debug_img, f"{label}: {size_text}",
                        (x1, y_t),#(x1, max(0, y2 + 15)),
                        font, font_scale, colour, thickness)

            # ----------------------------------------------------------------------
            # 6) Fill vision_msgs/Detection3D
            # ----------------------------------------------------------------------
            det3d = Detection3D()

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label
            hypo.hypothesis.score = conf
            det3d.results.append(hypo)

            # 3D center of object (use centroid)
            det3d.bbox.center.position.x = float(center[0])
            det3d.bbox.center.position.y = float(center[1])
            det3d.bbox.center.position.z = float(center[2])

            det3d.bbox.size.x = float(obj_W)
            det3d.bbox.size.y = float(obj_H)
            det3d.bbox.size.z = float(obj_H if shape == "round" else obj_D)

            det_array.detections.append(det3d)

        cv2.imshow("Resultant Image", debug_img)
        cv2.waitKey(1)
        # Publish outputs
        self.pub_det.publish(det_array)
        self.pub_debug.publish(self.ndarray_to_image_msg(debug_img, rgb_msg))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
