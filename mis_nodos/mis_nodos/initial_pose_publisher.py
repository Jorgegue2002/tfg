#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            1
        )

        # Construye el mensaje
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Posición en la que sspawnea el robot
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 1.0
        msg.pose.pose.position.z = 0.0

        # Sin rotación (Modulo de los cuatro = 1)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Covarianza (6×6 aplanada) con confianza alta en posición y orientación
        msg.pose.covariance = [
            0.0025, 0,      0,      0,      0,      0,
            0,      0.0025, 0,      0,      0,      0,
            0,      0,      0.0001, 0,      0,      0,
            0,      0,      0,      0.0001, 0,      0,
            0,      0,      0,      0,      0.0001, 0,
            0,      0,      0,      0,      0,      0.0004
        ]

        # Publica una vez tras x segundos para asegurarnos de que AMCL ya esté activo
        self.create_timer(10.0, self.publish_initial_pose, callback_group=None)
        self.msg = msg

    def publish_initial_pose(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publicado initialpose en (0.0,1.0)')
        self.destroy_node()  # Fin de este nodo tras publicar

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
