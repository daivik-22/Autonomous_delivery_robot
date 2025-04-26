#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Try hardware; otherwise simulate
try:
    from mfrc522 import SimpleMFRC522
    REAL_HW = True
except (RuntimeError, ModuleNotFoundError):
    REAL_HW = False

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')
        self.pub = self.create_publisher(Bool, 'item_loaded', 10)
        self.get_logger().info('RFID mode: ' + ('HW' if REAL_HW else 'SIM'))
        self.create_timer(0.1, self.spin)

    def spin(self):
        if REAL_HW:
            uid, _ = SimpleMFRC522().read()
            self.get_logger().info(f'Tag read: {uid}')
            self.pub.publish(Bool(data=True))
        else:
            now = time.time()
            if not hasattr(self, '_last'):
                self._last = now
            if now - self._last > 5.0:
                self._last = now
                self.get_logger().info('SIM: Publishing fake tag read')
                self.pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = RFIDReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
