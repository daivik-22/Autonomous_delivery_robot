#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__('health_monitor_node')
        self.sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, 10)
        self.alert_pub = self.create_publisher(String, 'bot/alert', 10)
        self.last_pos = None
        self.stuck_count = 0
        self.get_logger().info('HealthMonitorNode listening on /odom')

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pos = (round(x, 3), round(y, 3))
        if pos == self.last_pos:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        self.last_pos = pos

        if self.stuck_count > 5:
            alert = String(data=f'Bot stuck at {pos}')
            self.alert_pub.publish(alert)
            self.get_logger().warn(alert.data)

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
