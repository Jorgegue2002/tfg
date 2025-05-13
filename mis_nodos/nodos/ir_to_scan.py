import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Header
import math
import time

class IRToLaserScanNode(Node):
    def __init__(self):
        super().__init__('ir_to_scan')

        # Ángulos de los sensores en radianes
        self.sensor_angles_deg = [-65.3, -38, -20, -3, 14, 25, 34]
        self.sensor_angles_rad = [math.radians(a) for a in self.sensor_angles_deg]

        # Parámetros del LaserScan
        self.range_min = 0.025
        self.range_max = 0.2
        self.frame_id = 'base_link'  # o el frame real de tu robot

        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.ir_callback,
            10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

    def ir_callback(self, msg: IrIntensityVector):
        scan = LaserScan()

        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = min(self.sensor_angles_rad)
        scan.angle_max = max(self.sensor_angles_rad)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(self.sensor_angles_rad) - 1)
        scan.scan_time = 1.0 / 17.5  # Tasa medida ~17.5 Hz. Medido mediante: ros2 topic hz /ir_intensity
        scan.time_increment = scan.scan_time / (len(self.sensor_angles_rad) - 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Convertir intensidad a distancia estimada (simple: inverso lineal)
        # Suponemos intensidad entre 0 (lejos) y 1000 (muy cerca)
        #
        ranges = []
        for reading in msg.readings:
            intensity = reading.value
            if intensity <= 0:
                distance = scan.range_max
            else:
                distance = max(self.range_min, min(self.range_max, 0.5 / intensity))
            ranges.append(distance)

        scan.ranges = ranges

        #Esto es opcional
        #scan.intensities = [r.value for r in msg.readings]

        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = IRToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
