#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix

class GPSFrameFixer(Node):
    def __init__(self):
        super().__init__('gps_frame_fixer')

        # Launch may already declare use_sim_time; avoid duplicate declaration
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        sensor_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Subscribe to original GPS topic
        self.sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.callback,
            sensor_qos
        )

        # Publish corrected GPS messages
        self.pub = self.create_publisher(
            NavSatFix,
            '/gps/fix_corrected',
            output_qos
        )

        self.get_logger().info(
            "GPS Frame Fixer active: /gps/fix → /gps/fix_corrected (frame_id = 'gps_link')"
        )

    def callback(self, msg):
        # Overwrite frame ID and stamp with simulation time so downstream nodes get consistent timing
        msg.header.frame_id = 'gps_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
