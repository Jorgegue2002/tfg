#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class MuxOutToStamped(Node):
    def __init__(self):
        super().__init__('mux_out_to_cmd_vel')
        # Suscríbete al output de twist_mux
        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel_out', 
            self.cb_twist, 
            10)
        # Publica el TwistStamped que necesita tu robot
        self.pub = self.create_publisher(
            TwistStamped, 
            '/cmd_vel', 
            10)

    def cb_twist(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = msg
        self.pub.publish(ts)

def main():
    rclpy.init()
    node = MuxOutToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Solo llama a shutdown si no se ha hecho ya
            rclpy.shutdown()


if __name__ == '__main__':
    main()

