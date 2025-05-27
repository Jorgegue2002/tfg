import rclpy
from rclpy.node import Node
from interfaces.msg import Envio
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class Entregador(Node):
    def __init__(self):
        super().__init__('entregador')
        self.destinos = { 
            #Para darles coordenadas en el mapa a cada numero
            #1: (10.0, 0.0, 0.0),
        }
        self.pub_envio = self.create_publisher(Envio, '/envio', 10)
        self.sub_envio = self.create_subscription(Envio, '/envio', self.cb_envio, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.estado = None

    def cb_envio(self, msg: Envio):
        # Para reaccionar a los cambios
        if msg.state == self.estado and msg.destination == getattr(self, 'dest', None):
            return
        self.estado = msg.state
        self.dest = msg.destination

        if msg.state == 'on' and msg.destination in self.destinos:
            self.ir_a(msg.destination)
        elif msg.state == 'off':
            self.ir_a(0)       # El 0 es el dock/lugar de salida

    def ir_a(self, destino):
        x, y, yaw = self.destinos.get(destino, (0.0, 0.0, 0.0))
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.ac.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        self.get_logger().info(f'→ Enviando a destino {destino}')
        send_goal = self.ac.send_goal_async(goal_msg, feedback_callback=self._fb)
        send_goal.add_done_callback(lambda f: self._on_result(f, destino))

    def _fb(self, feedback):
        p = feedback.feedback.current_pose.pose.position
        self.get_logger().info(f'  avanzando… ({p.x:.2f}, {p.y:.2f})')

    def _on_result(self, future, destino):
        result = future.result().result
        if result.successful:
            self.get_logger().info(f'Destino {destino} alcanzado')
            nuevo = Envio()
            nuevo.state = 'waiting_for_discharge' if destino != 0 else 'waiting_for_order'
            nuevo.destination = destino
            self.pub_envio.publish(nuevo)
        else:
            self.get_logger().warn('No se ha podido alcanzar el destino')

def main(args=None):
    rclpy.init(args=args)
    n = Entregador()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
