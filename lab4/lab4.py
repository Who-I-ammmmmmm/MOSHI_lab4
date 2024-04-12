import random
import numpy as np
import csv
import matplotlib.pyplot as plt

# Генерация карты дорог
def generate_route_map(num_cities):
    route_map = {}

    for i in range(num_cities):
        distances = {}  
        for j in range(num_cities+1):
            if i == j:
                distances[j] = 0
            else:
                distances[j] = random.randint(10, 100)
        route_map[i] = distances

    return route_map

# Сохранение карты дорог
def save_route_map(route_map, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for key, value in route_map.items():
            writer.writerow([key] + list(value.values()))

# Загрузка карты дорог
def load_route_map(filename):
    route_map = {}
    with open(filename, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            route_map[int(row[0])] = {i: int(row[i+1]) for i in range(len(row)-1)}
    return route_map

# Реализация симуляции колонии муравьев
class AntColony:
    def __init__(self, route_map, num_ants, evaporation_rate, alpha, beta):
        self.route_map = route_map
        self.num_ants = num_ants
        self.evaporation_rate = evaporation_rate
        self.alpha = alpha
        self.beta = beta
        self.num_cities = len(route_map)
        self.pheromone = np.ones((self.num_cities, self.num_cities))
        self.global_best_path = None
        self.global_best_distance = float('inf')

    def run(self, num_iterations):
        for _ in range(num_iterations):
            local_best_path = None
            local_best_distance = float('inf')

            for ant in range(self.num_ants):
                path, distance = self.ant_path()
                if distance < local_best_distance:
                    local_best_path = path
                    local_best_distance = distance

            if local_best_distance < self.global_best_distance:
                self.global_best_path = local_best_path
                self.global_best_distance = local_best_distance

            self.update_pheromone(local_best_path, local_best_distance)

    def ant_path(self):
        start_city = random.randint(0, self.num_cities - 1)
        visited = [False] * self.num_cities
        visited[start_city] = True
        path = [start_city]
        distance = 0

        for _ in range(self.num_cities - 1):
            next_city = self.choose_next_city(path, visited)
            path.append(next_city)
            visited[next_city] = True
            distance += self.route_map[path[-2]][next_city]

        distance += self.route_map[path[-1]][start_city]
        return path, distance

    def choose_next_city(self, path, visited):
        current_city = path[-1]
        unvisited_cities = [i for i in range(self.num_cities) if not visited[i]]
        probabilities = []

        for city in unvisited_cities:
            pheromone = self.pheromone[current_city][city]
            visibility = 1 / self.route_map[current_city][city]
            probabilities.append((pheromone ** self.alpha) * (visibility ** self.beta))

        total_prob = sum(probabilities)
        probabilities = [prob / total_prob for prob in probabilities]

        return np.random.choice(unvisited_cities, p=probabilities)

    def update_pheromone(self, path, distance):
        for i in range(len(path) - 1):
            city1, city2 = path[i], path[i+1]
            self.pheromone[city1][city2] = (1 - self.evaporation_rate) * self.pheromone[city1][city2] + (1 / distance)

    def get_global_best_path(self):
        return self.global_best_path

    def get_global_best_distance(self):
        return self.global_best_distance


# Отображение на графике
# Отображение на графике
def plot(best_path):
    x_coords = []
    y_coords = []
    for city in route_map:
        x_coords.append(route_map[city][0])
        y_coords.append(route_map[city][1])

    plt.figure(figsize=(8, 6))
    plt.scatter(x_coords, y_coords, color='blue', label='Cities')
    plt.scatter(x_coords[best_path[0]], y_coords[best_path[0]], color='pink', label='Start')
    plt.scatter(x_coords[best_path[-1]], y_coords[best_path[-1]], color='aqua', label='End')

    # Рисуем направление пути с помощью стрелок
    for i in range(len(best_path) - 1):
        city1_index = best_path[i]
        city2_index = best_path[i+1]
        plt.arrow(x_coords[city1_index], y_coords[city1_index], 
                  x_coords[city2_index] - x_coords[city1_index], y_coords[city2_index] - y_coords[city1_index], 
                  color='red', linestyle='-', linewidth=2, head_width=2, length_includes_head=True)

    # Соединяем последний город с начальным, чтобы закрыть путь
    plt.arrow(x_coords[best_path[-1]], y_coords[best_path[-1]], 
              x_coords[best_path[0]] - x_coords[best_path[-1]], y_coords[best_path[0]] - y_coords[best_path[-1]], 
              color='red', linestyle='-', linewidth=2, head_width=2, length_includes_head=True)

    plt.title(f'Route Map with Best Path\n distance:')
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # Параметры симуляции
    # Количество городов
    num_cities = 30
    # Количество муравьев
    num_ants = 50
    # Коофицент испорения ферамонов
    evaporation_rate = 0.1
    # Вес - превликательность ферамонов
    alpha = 1
    # Количество - качество ферамонов
    beta = 2
    # Количество эпох
    num_iterations = 10
    # Загрузить старую карту или сгенерировать новую
    load_old_map = True

    print(f"[MAIN] Settings: \n Cities: {num_cities} \n Ants: {num_ants} \n Evaporation rate: {evaporation_rate} \n Alpha: {alpha} \n Beta: {beta} \n Epoth: {num_iterations} \n Load old map: {load_old_map} \n")

    if load_old_map == True:
        route_map = load_route_map('route_map')
    else:
        # Генерация карты маршрутов и сохранение её в файл
        route_map = generate_route_map(30)
        save_route_map(route_map, 'route_map')

    # Создание колонии муравьев и запуск симуляции
    colony = AntColony(route_map, num_ants, evaporation_rate, alpha, beta)
    colony.run(num_iterations)

    # Получение результатов симуляции
    best_path = colony.get_global_best_path()
    best_distance = colony.get_global_best_distance()

    print("[MAIN] Best Path:", best_path)
    print("[MAIN] Best Distance:", best_distance)
    plot(best_path)
