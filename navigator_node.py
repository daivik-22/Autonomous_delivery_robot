#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from autobot_pkg.navigator import Navigator

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        grid = [
            [0, 0, 0],
            [1, 1, 0],
            [0, 0, 0],
        ]
        self.nav = Navigator(grid)
        self.goal_sub = self.create_subscription(String, 'nav_goal', self.on_goal, 10)
        self.done_pub = self.create_publisher(Bool, 'nav_done', 10)
        self.get_logger().info('NavigatorNode ready. Awaiting nav_goal...')

    def on_goal(self, msg: String):
        data = json.loads(msg.data)
        start, goal = tuple(data['start']), tuple(data['goal'])
        self.get_logger().info(f'Planning path from {start} to {goal}')
        try:
            path = self.nav.compute_path(start, goal)
            self.get_logger().info(f'Path: {path}')
            self.nav.execute_path(path, delay=0.2)
            self.done_pub.publish(Bool(data=True))
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')
            self.done_pub.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
