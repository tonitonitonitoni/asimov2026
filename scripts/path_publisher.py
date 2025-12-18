#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)

        self.path = Path()
        self.path.header.frame_id = 'map'

    def odom_callback(self, msg):
        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = 'map'
        p.pose = msg.pose.pose

        self.path.poses.append(p)
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry, Path
# from geometry_msgs.msg import PoseStamped


# class PathPublisher(Node):
#     def __init__(self):
#         super().__init__("path_publisher")

#         self.path_pub = self.create_publisher(Path, "/robot_path", 10)

#         # subscribe to global odometry
#         self.create_subscription(
#             Odometry,
#             "/odometry/global",
#             self.odom_callback,
#             10
#         )

#         self.path = Path()
#         self.path.header.frame_id = "map"

#         self.get_logger().info("Robot path publisher started.")

#     def odom_callback(self, msg):
#         pose = PoseStamped()
#         pose.header = msg.header
#         pose.header.frame_id = "map"   # ensure consistent frame
#         pose.pose = msg.pose.pose

#         self.path.header.stamp = self.get_clock().now().to_msg()
#         self.path.poses.append(pose)
#         self.path.poses = self.path.poses[-2000:]  # keep last 2000 waypoints

#         self.path_pub.publish(self.path)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PathPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
