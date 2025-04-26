#!/usr/bin/env python3
import json
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        self.pub = self.create_publisher(String, 'bot/telemetry', 10)
        self.get_logger().info('TelemetryNode publishing on /bot/telemetry')
        self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        lat = 12.9000 + random.uniform(-0.0005, 0.0005)
        lon = 80.0000 + random.uniform(-0.0005, 0.0005)
        msg = String(data=json.dumps({'lat': lat, 'lon': lon}))
        self.pub.publish(msg)
        self.get_logger().info(f'Published telemetry: {lat:.6f}, {lon:.6f}')

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
