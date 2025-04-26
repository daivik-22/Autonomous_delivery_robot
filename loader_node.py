#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class LoaderNode(Node):
    def __init__(self):
        super().__init__('loader_node')
        self.sub = self.create_subscription(
            Bool, 'item_loaded', self.on_loaded, 10)
        self.pub = self.create_publisher(String, 'nav_goal', 10)
        self.start = (0, 0)
        self.goal  = (2, 3)
        self.get_logger().info('LoaderNode ready.')

    def on_loaded(self, msg: Bool):
        if msg.data:
            payload = {'start': self.start, 'goal': self.goal}
            js = json.dumps(payload)
            self.get_logger().info(f'Item loaded → publishing nav_goal: {js}')
            self.pub.publish(String(data=js))

def main(args=None):
    rclpy.init(args=args)
    node = LoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
