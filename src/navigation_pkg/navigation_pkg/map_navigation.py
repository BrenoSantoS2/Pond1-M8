import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
from queue import PriorityQueue
import numpy as np  # Import necessário para trabalhar com arrays

class MapNavigation(Node):
    def __init__(self):
        super().__init__('map_navigation')
        
        # Cliente para o serviço de movimentação
        self.move_client = self.create_client(MoveCmd, '/move_command')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /move_command não disponível, tentando novamente...')
        self.get_logger().info('Serviço /move_command disponível.')
        
        # Cliente para o serviço de obtenção do mapa
        self.map_client = self.create_client(GetMap, '/get_map')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /get_map não disponível, tentando novamente...')
        self.get_logger().info('Serviço /get_map disponível.')

    def send_move_request(self, direction):
        """Envia uma solicitação de movimento para o robô."""
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_map(self):
        """Obtém o mapa do serviço /get_map."""
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        # Reconstrói o grid do mapa
        grid_flat = response.occupancy_grid_flattened
        shape = response.occupancy_grid_shape
        grid = [grid_flat[i * shape[1]:(i + 1) * shape[1]] for i in range(shape[0])]
        return grid

    def get_direction(self, current, next_pos):
        """Determina a direção de movimento com base na posição atual e no próximo passo."""
        dx = next_pos[0] - current[0]
        dy = next_pos[1] - current[1]
        if dx == -1: return 'up'
        if dx == 1: return 'down'
        if dy == -1: return 'left'
        if dy == 1: return 'right'
        return None

    def a_star(self, grid, start, goal):
        """Planeja o caminho usando o algoritmo A*."""
        def heuristic(pos1, pos2):
            return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
        
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            neighbors = self.get_neighbors(grid, current)
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
        return None

    def get_neighbors(self, grid, position):
        """Obtém os vizinhos válidos de uma posição no grid."""
        neighbors = []
        x, y = position
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # direita, baixo, esquerda, cima
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != 'b':
                neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstrói o caminho do objetivo até o ponto inicial."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def navigate_with_map(self, grid, robot_pos, target_pos):
        """Navega até o destino usando A*."""
        self.get_logger().info(f'Iniciando navegação de {robot_pos} para {target_pos}')
        
        path = self.a_star(grid, tuple(robot_pos), tuple(target_pos))
        if not path:
            self.get_logger().error('Nenhum caminho encontrado para o alvo!')
            return False

        self.get_logger().info(f'Caminho planejado: {path}')
        for step in path:
            direction = self.get_direction(robot_pos, step)
            result = self.send_move_request(direction)
            robot_pos = result.robot_pos
            
            # Atualização para evitar problemas de array
            if not np.array_equal(robot_pos, step):
                self.get_logger().error(f'Erro ao mover para {step}. Posição atual: {robot_pos}')
                return False

            self.get_logger().info(f'Movendo para {robot_pos}')
        
        # Verifica a posição final
        if not np.array_equal(robot_pos, target_pos):
            self.get_logger().error(f'O robô parou em {robot_pos} ao invés de {target_pos}')
            return False

        self.get_logger().info('Navegação concluída com sucesso!')
        return True


def main():
    rclpy.init()
    node = MapNavigation()

    # Obter o mapa e configurações
    grid = node.get_map()
    robot_pos = np.array([2, 2])  # Pos inicial
    target_pos = np.array([17, 17])  # Alvo no mapa

    if not node.navigate_with_map(grid, robot_pos, target_pos):
        node.get_logger().error('Falha na navegação.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
