#!/usr/bin/env python3
"""
map→odom TF Publisher for nvblox Navigation

This node publishes map→odom TF transformation for navigation systems
using nvblox.

Operating Principle:
1. nvblox publishes /nvblox_node/static_occupancy_grid in map frame
2. ZED Visual Odometry publishes odom→zed_camera_link TF
3. This node publishes map→odom TF to complete the TF tree

Initially publishes identity transform (map = odom),
which can be corrected in the future with SLAM loop closure, etc.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class MapOdomTfPublisher(Node):
    def __init__(self):
        super().__init__('map_odom_tf_publisher')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_static_transform', True)  # Use static transform

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_static_transform = self.get_parameter('use_static_transform').value

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initial transform (identity transform: map = odom)
        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = self.map_frame
        self.map_to_odom.child_frame_id = self.odom_frame
        self.map_to_odom.transform.translation.x = 0.0
        self.map_to_odom.transform.translation.y = 0.0
        self.map_to_odom.transform.translation.z = 0.0
        self.map_to_odom.transform.rotation.x = 0.0
        self.map_to_odom.transform.rotation.y = 0.0
        self.map_to_odom.transform.rotation.z = 0.0
        self.map_to_odom.transform.rotation.w = 1.0

        # Odometry subscriber (for future expansion)
        # Currently only static transform, but can monitor odom for correction in the future
        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )

        # TF publishing timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_tf)

        self.get_logger().info(
            f'map→odom TF Publisher started\n'
            f'  Map frame: {self.map_frame}\n'
            f'  Odom frame: {self.odom_frame}\n'
            f'  Publish rate: {self.publish_rate} Hz\n'
            f'  Mode: {"Static (Identity)" if self.use_static_transform else "Dynamic"}'
        )

    def odom_callback(self, msg: Odometry):
        """
        Odometry callback (for future expansion)

        Currently only static transform, but can implement in the future:
        - Loop closure detection
        - Global pose estimation
        - Drift correction
        etc.
        """
        pass

    def publish_tf(self):
        """Publish map→odom TF"""
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_to_odom)

def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
