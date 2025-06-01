import rclpy
from rclpy.node import Node
from interfaces.msg import Envio
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

class Entregador(Node):
    def __init__(self):
        super().__init__('entregador')

        # self.initial_x = 0.3  
        # self.initial_y = -17.0  

        self.destinos = { 
            #Para darles coordenadas en el mapa a cada numero
            0: (0.3, -17.0, 3.14),
            1: (8.0, -5.0, 0.0),
            2: (4.0, 2.0, 0.0)
        }
        self.pub_envio = self.create_publisher(Envio, '/envio', 10)
        self.sub_envio = self.create_subscription(Envio, '/envio', self.cb_envio, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.estado = None
        self.dest = None

    def cb_envio(self, msg: Envio):
        # Para reaccionar a los cambios
        self.get_logger().info(f'Recibido mensaje: status={msg.status}, destino={msg.destination}')
        if msg.status == self.estado and msg.destination == getattr(self, 'dest', None):
            return
        self.estado = msg.status
        self.dest = msg.destination

        self.get_logger().info('→ Cambiando estado y procesando...')

        if msg.status == 'on' and msg.destination in self.destinos:
            self.ir_a(msg.destination)
        elif msg.status == 'off':
            self.ir_a(0)       # El 0 es el dock/lugar de salida

    def ir_a(self, destino):
        # # La posicion relativa
        # dx, dy, yaw = self.destinos.get(destino, (0.0, 0.0, 0.0))
        # # Le restamos la posicion inicial
        # x = dx - self.initial_x
        # y = dy - self.initial_y

        x, y, yaw = self.destinos.get(destino,(0.0, 0.0, 0.0))

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.get_logger().info('Esperando a Nav2...')  
        self.ac.wait_for_server()
        self.get_logger().info('Servidor Nav2 disponible.')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.get_logger().info(f'→ Enviando a destino {destino}')
        send_goal_future = self.ac.send_goal_async(goal_msg, feedback_callback=self._fb)  
        send_goal_future.add_done_callback(lambda future: self._on_goal_response(future, destino))

    def _fb(self, feedback):
        p = feedback.feedback.current_pose.pose.position
        self.get_logger().info(f'  avanzando… ({p.x:.2f}, {p.y:.2f})')

    def _on_goal_response(self, future, destino):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rechazado para destino {destino}')
            return
        self.get_logger().info(f'Goal aceptado para destino {destino}, esperando resultado...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future_result: self._on_result(future_result, destino))

    def _on_result(self, future, destino):
        resp = future.result()
        if resp.status == GoalStatus.STATUS_SUCCEEDED:
            error = resp.result.error_code
            self.get_logger().info(f'Destino {destino} alcanzado (error_code={error})')
            nuevo = Envio()
            nuevo.status = 'waiting_for_discharge' if destino != 0 else 'waiting_for_order'
            nuevo.destination = destino
            self.pub_envio.publish(nuevo)
        else:
            error = resp.result.error_code
            self.get_logger().warn(
                f'No se ha podido alcanzar el destino {destino} '
                f'(status={resp.status}, error_code={error})'
            )

def main(args=None):
    rclpy.init(args=args)
    n = Entregador()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
