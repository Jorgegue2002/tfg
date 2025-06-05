import rclpy
from rclpy.node import Node
from interfaces.msg import Envio
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

class Entregador(Node):
    def __init__(self):
        super().__init__('entregador')

        # Nodo "entregador": gestiona misiones de entrega en un entorno simulado con ROS 2 y Nav2.
        # Recibe pedidos por `/envio`, navega a los destinos y espera la confirmación en `/robot_ready`.
        # Soporta entregas simples y combinadas (pickup and deliver) con gestión automática de cola.
        # Para uso, publicar mensajes en `/envio` con status y destino; el robot se encargará del resto.

        # Los diferentes tipos de status y los destinos se puede comprobar en interfaces/msg/Envio.msg


        #Los destinos son un numero del 1 al 9, y en este diccionario se recogen a que coordenadas (de Gazebo) corresponde cada numero
        self.destinos_gz = { 
            #Para darles coordenadas en el mapa a cada numero
            0: (0.3, -17.0, 3.14),
            1: (10.0, -4.0, 0.0),
            2: (10.0, -18.0, 0.0),
            3: (4.0, -30.0, 0.0),
            4: (-10.0, -18.0, 0.0),
            5: (-10.0, -4.0, 0.0),
            6: (3.0, -15.0, 0)
        }

        # Tranformar las coordenadas de Gazebo en las coordenadas del mapa
        def gazebo_a_mapa(destinos_gz):
            origen_gz = (0.3, -17)  # Origen del mapa en coordenadas de Gazebo
            destinos_map = {}

            for i, (x_gz, y_gz, theta) in destinos_gz.items():
                x_map = -(x_gz - origen_gz[0])
                y_map = -(y_gz - origen_gz[1])
                destinos_map[i] = (x_map, y_map, theta)

                self.get_logger().info(
                f'Destino {i} | Gazebo: ({x_gz:.2f}, {y_gz:.2f}) → Mapa: ({x_map:.2f}, {y_map:.2f})'
                )

            return destinos_map

        self.destinos_mapa = gazebo_a_mapa(self.destinos_gz)

        self.pub_envio = self.create_publisher(Envio, '/envio', 10)
        self.sub_envio = self.create_subscription(Envio, '/envio', self.cb_envio, 10)
        self.ready_pub = self.create_publisher(Bool, '/robot_ready', 10)
        self.ready_sub = self.create_subscription(Bool, '/robot_ready', self._ready_cb, 10)

        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.estado = None
        self.dest = None
        self.entregando = False
        self.cola = []

        #Parametros del temporizador de los waiting
        self.delay_descarga = 30.0 # Lo que espera cuando llega a un destino
        self._wait_timer = None

        #Parametros del temporizador del feedback
        self._last_fb_log = self.get_clock().now()
        self._fb_interval = rclpy.duration.Duration(seconds=5)


    def cb_envio(self, msg: Envio):
        # Para reaccionar a los cambios
        self.get_logger().info(f'Recibido mensaje: status={msg.status}, destino={msg.destination}')
        if msg.status == self.estado and msg.destination == getattr(self, 'dest', None):
            return
        self.estado = msg.status
        self.dest = msg.destination

        self.get_logger().info('Procesando...')

        if msg.status == 'deliver' and msg.destination in self.destinos_mapa:
            # Si ya está entregando, se añadira a la cola
            if self.entregando:
                self.get_logger().info(f'En proceso de entrega. Se añadio destino {msg.destination} a la cola.')
                self.cola.append(msg.destination)
                return
            # Si esta libre, inicia la entrega
            self.entregando = True
            self.ir_a(msg.destination)

        elif msg.status == 'pickup_and_deliver' and msg.destination in self.destinos_mapa:
            # Primero se va al dock y luego se mete en la cola ir al destino
            self.get_logger().info(f'En proceso de entrega. Se añadio a la cola el ir al dock y el destino {msg.destination}')
            if self.entregando:
                self.cola.append(0)
                self.cola.append(msg.destination)
            else:
                self.entregando = True
                self.ir_a(0)
                self.cola.append(msg.destination)

        elif msg.status == 'go_to_dock': #Destino 0 independientemente del que tenga en el mensaje
            # Lo mismo que para deliver, pero para volver al dock
            if self.entregando:
                self.get_logger().info('En proceso de entrega. Se añadira ir al dock a la cola.')
                self.cola.append(0)
                return
            self.entregando = True
            self.ir_a(0)

    def ir_a(self, destino):

        x, y, yaw = self.destinos_mapa.get(destino,(0.0, 0.0, 0.0))

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.get_logger().info('Esperando a Nav2...')  
        self.ac.wait_for_server()
        # self.get_logger().info('Servidor Nav2 disponible.')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.get_logger().info(f'→ Enviando a destino {destino}')
        send_goal_future = self.ac.send_goal_async(goal_msg, feedback_callback=self._fb)  
        send_goal_future.add_done_callback(lambda future: self._on_goal_response(future, destino))

    def _fb(self, feedback):
        p = feedback.feedback.current_pose.pose.position
        now = self.get_clock().now()
        # Solo loguear cada 5 segundos
        if now - self._last_fb_log >= self._fb_interval:
            self.get_logger().info(f'  avanzando…. Posición del mapa: ({p.x:.2f}, {p.y:.2f})')
            self._last_fb_log = now


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
            self.get_logger().info(f'Destino {destino} alcanzado')

            # Cambia el estado a waiting
            nuevo = Envio()
            nuevo.status = 'waiting_for_discharge' if destino != 0 else 'waiting_for_order'
            nuevo.destination = destino
            self.pub_envio.publish(nuevo)

            # Inicia el temporizador para /robot_ready
            self.get_logger().info(f'Inicio del temporizador de {self.delay_descarga:.1f}s...')
            if self._wait_timer is not None:
                self._wait_timer.cancel()
            self._wait_timer = self.create_timer(self.delay_descarga, self._timeout_callback)

        else:
            error = resp.result.error_code
            self.get_logger().warn(
                f'No se ha podido alcanzar el destino {destino} '
                f'(status={resp.status}, error_code={error})'
            )
            self.entregando = False
            self._procesar_siguiente_en_cola()
        
    def _timeout_callback(self):
        # Se llama cuando expira el temporizador de espera
        self.get_logger().info('El temporizador ha expirado. Se publicara /robot_ready automaticamente.')
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

        # Publica el mensaje “ready” en /robot_ready
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

        # # Deja que el siguiente envio en la cola comience
        # self.entregando = False
        # self._procesar_siguiente_en_cola()

    def _ready_cb(self, msg: Bool):
        # Por si llega la señal /robot_ready
        if msg.data:
            self.get_logger().info('Señal /robot_ready recibida.')
            # Si el temporizador seguía activo, cancelarlo
            if self._wait_timer is not None:
                self._wait_timer.cancel()
                self._wait_timer = None
            # Deja que comience el siguiente envio de la cola
            self.entregando = False
            self._procesar_siguiente_en_cola()

    
    def _procesar_siguiente_en_cola(self):
        # Si hay destinos pendientes en la cola saca el primero de la cola y lo envia
        if not self.cola:
            return

        siguiente = self.cola.pop(0)
        self.get_logger().info(f'Procesando siguiente envio en cola: destino {siguiente}')

        self.estado = 'Procesando entrega en cola'
        self.dest = siguiente
        self.entregando = True
        self.ir_a(siguiente)

def main(args=None):
    rclpy.init(args=args)
    n = Entregador()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
