import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
from cg_interfaces.srv import GetMap
import random

class ReactiveNavigation(Node):

    def __init__(self):
        super().__init__('reactive_navigation')
        self.client = self.create_client(MoveCmd, '/move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /move_command não disponível, tentando novamente...')
        self.get_logger().info('Serviço /move_command disponível.')

    def send_move_request(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_towards_target(self, robot_pos, target_pos, directions):
        """Simples função para movimento reativo. O robô tenta se mover para o alvo."""
        if robot_pos == target_pos:
            return True

        possible_moves = []
        for direction in directions:
            if direction == 'left' and directions['left'] != 'b':
                possible_moves.append('left')
            elif direction == 'right' and directions['right'] != 'b':
                possible_moves.append('right')
            elif direction == 'up' and directions['up'] != 'b':
                possible_moves.append('up')
            elif direction == 'down' and directions['down'] != 'b':
                possible_moves.append('down')

        if possible_moves:
            direction = random.choice(possible_moves)  # Move aleatório entre opções válidas
            self.send_move_request(direction)
            return False
        return False


def main():
    rclpy.init()
    node = ReactiveNavigation()
    directions = {
        'left': 'f',  # Em cada direção, adicione o valor que você obteve do serviço
        'down': 'f',
        'up': 'f',
        'right': 'f'
    }

    robot_pos = [0, 0]
    target_pos = [18, 18]  # Defina a posição do alvo

    # Exemplo de movimentação até o alvo
    while not node.move_towards_target(robot_pos, target_pos, directions):
        rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
