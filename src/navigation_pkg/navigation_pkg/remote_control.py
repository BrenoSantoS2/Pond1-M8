import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd  # Substitua "cg" pelo nome do seu pacote

class MoveRobotClient(Node):

    def __init__(self):
        super().__init__('move_robot_client')
        self.client = self.create_client(MoveCmd, '/move_command')

        # Espera o serviço estar disponível
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /move_command não disponível. Tentando novamente...')

    def send_move_request(self, direction):
        # Cria uma requisição de movimento
        request = MoveCmd.Request()
        request.direction = direction

        # Envia a requisição e espera a resposta
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = MoveRobotClient()

    # Exemplo de movimento
    direction = "up"  # pode ser "left", "down", "up", ou "right"
    response = client.send_move_request(direction)

    if response.success:
        print(f"Movimento bem-sucedido! Nova posição do robô: {response.robot_pos}")
        print(f"Posição alvo: {response.target_pos}")
    else:
        print("Movimento falhou.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
