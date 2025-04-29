import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class Entregador(Node):
    def __init__(self):
        super().__init__('entregador_simple')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Coordenadas destino
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 2.0
        self.goal_pose.pose.position.y = 1.5
        self.goal_pose.pose.orientation.w = 1.0  #Rotacion

        self.send_goal()

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self.get_logger().info('Enviando objetivo...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Objetivo rechazado')
            return

        self.get_logger().info('Objetivo aceptado')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Avanzando: {feedback.current_pose.pose.position}')

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Mision cumplida')

def main(args=None):
    rclpy.init(args=args)
    node = Entregador()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
