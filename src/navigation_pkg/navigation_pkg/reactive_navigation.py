import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import random


class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation')
        self.client = self.create_client(MoveCmd, '/move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /move_command não disponível, tentando novamente...')
        self.get_logger().info('Serviço /move_command disponível.')
        self.visited_positions = set()  # Conjunto para armazenar posições visitadas

    def send_move_request(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_towards_target(self, robot_pos, target_pos, directions):
        """Movimento reativo com registro de pontos visitados."""
        if tuple(robot_pos) == tuple(target_pos):  # Se o robô já estiver no alvo
            self.get_logger().info('Alvo alcançado!')
            return True

        # Adiciona a posição atual à lista de visitados
        self.visited_positions.add(tuple(robot_pos))

        possible_moves = []
        new_positions = {
            'left': (robot_pos[0], robot_pos[1] - 1),
            'right': (robot_pos[0], robot_pos[1] + 1),
            'up': (robot_pos[0] - 1, robot_pos[1]),
            'down': (robot_pos[0] + 1, robot_pos[1]),
        }

        # Filtra movimentos válidos e que não levam a posições visitadas
        for direction, new_pos in new_positions.items():
            if direction in directions and directions[direction] != 'b' and new_pos not in self.visited_positions:
                possible_moves.append(direction)

        if possible_moves:
            # Escolhe um movimento válido aleatório
            direction = random.choice(possible_moves)
            response = self.send_move_request(direction)

            if response.success:
                # Atualiza a posição do robô
                robot_pos[0], robot_pos[1] = new_positions[direction]
                self.get_logger().info(f'Movendo para {direction}. Nova posição: {robot_pos}')
            return False

        # Caso todos os movimentos válidos já tenham sido visitados, faz um movimento aleatório
        fallback_moves = [d for d, pos in new_positions.items() if d in directions and directions[d] != 'b']
        if fallback_moves:
            direction = random.choice(fallback_moves)
            response = self.send_move_request(direction)
            if response.success:
                robot_pos[0], robot_pos[1] = new_positions[direction]
                self.get_logger().info(f'Movimento de fallback para {direction}. Nova posição: {robot_pos}')
            return False

        self.get_logger().warning('Nenhum movimento possível.')
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

    robot_pos = [2, 2]
    target_pos = [18, 18]  # Defina a posição do alvo

    # Exemplo de movimentação até o alvo
    while not node.move_towards_target(robot_pos, target_pos, directions):
        rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
