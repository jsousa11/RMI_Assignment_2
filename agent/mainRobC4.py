import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from math import inf
import numpy as np
import itertools
from bfs import bfs

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.posList = []
        self.errList = []
        self.counter = 0
        self.counterrot = 0
        self.countergps = 0
        self.counterfree = 0
        self.length = 2
        self.lengthrot = 2
        self.rotation_target = 0
        self.endCycle = True
        self.onRot = False
        self.reversing = False
        self.South = False
        self.maze = Lab()
        self.last_x = 27
        self.last_y = 13
        self.unknown_cells = []
        self.known_cells = []
        self.path = []
        self.searching = False
        self.track_visited_cells = [(0, 0)]
        self.pathfollowing = False
        self.haspath = False
        self.beacon_coordinates = [(0, 0)]
        self.beacon_nums = []
        self.go_to_beacons = False
        self.f = None
        self.final_path = []
        self.estimated_velocity = [(0, 0)]
        self.position_history = [(0, 0)]
        self.WallClose = False
        self.side_correction = False
        self.left_detected = False
        self.right_detected = False
        self.return_to_start = False
        self.first = True

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed == True:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    # Função para fazer o robô andar
    def wander(self):
        self.measures.x = self.position_history[-1][0]
        self.measures.y = self.position_history[-1][1]
        center_sensor = self.measures.irSensor[0]
        left_sensor = self.measures.irSensor[1]
        right_sensor = self.measures.irSensor[2]
        back_sensor = self.measures.irSensor[3]
        self.checkChangeCompass()

        if self.South and self.measures.compass < -90:
            self.measures.compass += 360
        if self.endCycle:
            if not self.onRot:
                self.converter(0, 0)
                self.left_detected = left_sensor >= 1.5
                self.right_detected = right_sensor >= 1.5
            if self.onRot:
                self.left_detected = False
                self.right_detected = False
                self.onRot = self.rotate(0.5, 0, 0, self.rotation_target, False)
            elif self.searching:
                loc = self.round_even(self.measures.x), self.round_even(self.measures.y)

                if loc == self.path[0]:
                    self.path = self.path[1:]
                if len(self.path) == 0:
                    self.haspath = False
                    self.searching = False
                else:
                    x, y = (self.path[0][0] - loc[0]), (self.path[0][1] - loc[1])
                    current = self.corrected_compass()

                    if x < 0:
                        self.rotation_target = 180
                    elif x > 0:
                        self.rotation_target = 0
                    elif y < 0:
                        self.rotation_target = -90
                    elif y > 0:
                        self.rotation_target = 90
                    else:
                        self.onRot = False
                    if self.rotation_target != current:
                        self.onRot = True
                    else:
                        self.onRot = False
                        self.searching = False

            elif (center_sensor + back_sensor)/2 >= 1.0 and not self.pathfollowing:
                self.searchUnknown()
                self.find_free_direction()
                self.onRot = True
                self.first = False
            elif self.first:
                if self.corrected_compass() == 0:
                    self.onRot = True
                    self.rotation_target = 180
                    self.searchUnknown()
                elif self.corrected_compass() == 180:
                    self.onRot = True
                    self.rotation_target = 0
                    self.searchUnknown()
                    self.first = 0
            else:
                self.appendWalked()
                self.amknown = self.searchKnown()
                self.searchUnknown()

                if self.South:
                    self.South = False

                if self.amknown:
                    self.searching = True

                    if not self.haspath:
                        start = self.round_even(self.measures.x), self.round_even(self.measures.y)
                        end_list = self.unknown_cells

                        end = self.bfs(start, end_list)
                        if not end and self.return_to_start:
                            neighbours = [(-1, 0), (0, -1), (1, 0), (0, 1)]
                            min_end = None
                            min_path = None
                            for neighbour in neighbours:
                                if neighbour in self.known_cells:
                                    end = self.bfs(start, [neighbour])
                                    path = self.path
                                    if min_path:
                                        if len(path) < len(min_path):
                                            min_end = end
                                            min_path = path
                                    else:
                                        min_end = end
                                        min_path = path
                            end = min_end
                            self.path = min_path

                        self.path = [items for items in self.path if items[0] % 2 == 0 and items[1] % 2 == 0]
                        self.path.append((2 * end[0] - self.path[-1][0], 2 * end[1] - self.path[-1][1]))
                        self.path.remove(start)

                        self.pathfollowing = True
                        self.haspath = True
                    else:   
                        self.endCycle = False

                if not self.pathfollowing:
                    self.endCycle = False

        else:
            self.endCycle = self.moveFront(0.1, 0.01, 0.00005)

    # Função para usar o algoritmo de busca em largura
    def bfs(self, start, goal_list):
        min_len = inf
        min_idx = -1
        min_path = []

        for idx, goal in enumerate(goal_list):
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            for dx, dy in neighbors:
                neighbor = (goal[0] + dx, goal[1] + dy)

                if (self.maze.matrix[13 - neighbor[1]][neighbor[0] + 27] == 'X' or
                    self.maze.matrix[13 - neighbor[1]][neighbor[0] + 27] == 'I'):
                    path, timeout = bfs(self.maze.matrix, start, neighbor)
                    if timeout:
                        continue

                    length = len(path)
                    if length < min_len:
                        min_idx = idx
                        min_len = length
                        min_path = path

        self.path = min_path
        if goal_list:
            end = goal_list[min_idx]
            return end
        else:
            self.complete_mapping_and_path()

    def complete_mapping_and_path(self):
        if self.return_to_start:
            self.finish()
            sys.exit()

        initial_position = (0, 0)
        self.beacon_coordinates.remove(initial_position)
        permutations = list(itertools.permutations(self.beacon_coordinates))
        self.final_path = []

        for perm in permutations:
            path_segment = self.calculate_path(initial_position, perm)
            if not self.final_path or len(path_segment) < len(self.final_path):
                self.final_path = path_segment

        self.final_path.append(initial_position)
        self.final_path = [p for p in self.final_path if p[0] % 2 == 0 and p[1] % 2 == 0]
        self.writePath()
        self.update_beacons_on_map()

    def calculate_path(self, start, waypoints):
        path = []
        current_position = start
        for waypoint in waypoints:
            segment, _ = bfs(self.maze.matrix, current_position, waypoint)
            path.extend(segment[:-1])
            current_position = waypoint
        return path

    # Função para atualizar os beacons no mapa
    def update_beacons_on_map(self):
        self.maze.matrix[13][27] = '0'
        for beacon, num in zip(self.beacon_coordinates, self.beacon_nums):
            y = 13 - beacon[1]
            x = 27 + beacon[0]
            self.maze.matrix[y][x] = str(num)
        self.writeMap()

    # Função para escrever o mapa
    def writeMap(self):
        f = open(self.f + '.map', 'w+')

        for line in self.maze.matrix:
            for element in line:
                f.write(element)
            f.write('\n')
        f.close()

    # Função para escrever o caminho
    def writePath(self):
        i = 0
        f = open(self.f + '.path', 'w+')
        for x, y in self.final_path:
            f.write(str(x) + ' ' + str(y) + ' ')

            if (x, y) in self.beacon_coordinates and (x, y) != (0, 0):
                f.write('#' + str(i))
                i += 1
            f.write('\n')
        f.close()

    def moveFront(self, Kp, Kd, Ki):
        current = self.corrected_compass()
        xin, yin = self.round_even(self.measures.x), self.round_even(self.measures.y)

        if current == 0:
            self.initialize_movement(xin + 2, 0.14)
            err = self.obj - self.measures.x
        elif current == 90:
            self.initialize_movement(yin + 2, 0.14)
            err = self.obj - self.measures.y
        elif current == 180:
            self.initialize_movement(xin - 2, 0.14, reversing=True)
            err = -self.obj + self.measures.x
        elif current == -90:
            self.initialize_movement(yin - 2, 0.14, reversing=True)
            err = -self.obj + self.measures.y
        else:
            err = 0

        diff = err / self.lin if self.lin != 0 else 100
        self.integral += err
        self.lin = Kp * err + Kd * diff + Ki * self.integral
        self.length = err

        self.rotate(1, 0, 0, current, True)

        self.lin = min(self.lin, 0.14)
        self.rot = min(self.rot, 0.15)

        self.converter(self.lin, self.rot)
        self.counter += 1

        if -0.11 < self.length < 0.11:
            self.update_beacon_coordinates()
            self.counter = 0
            self.update_maze(current, xin, yin)
            return True
        return False

    def initialize_movement(self, obj_value, lin_value, reversing=False):
        if self.counter == 0:
            self.obj = obj_value
            self.lin = lin_value
            self.integral = 0
            self.reversing = reversing

    def update_beacon_coordinates(self):
        if self.measures.ground != -1:
            coord = (self.round_even(self.measures.x), self.round_even(self.measures.y))
            if coord not in self.beacon_coordinates:
                self.beacon_coordinates.append(coord)
                self.beacon_nums.append(self.measures.ground)

    def update_maze(self, current, xin, yin):
        if current == 0:
            self.detect_walls(0, self.last_x + 2, self.last_y)
            self.maze.matrix[self.last_y][self.last_x + 1] = 'X'
            self.maze.matrix[self.last_y][self.last_x + 2] = 'X'
            self.last_x = xin + 27
        elif current == 90:
            self.detect_walls(90, self.last_x, self.last_y - 2)
            self.maze.matrix[self.last_y - 1][self.last_x] = 'X'
            self.maze.matrix[self.last_y - 2][self.last_x] = 'X'
            self.last_y = -yin + 13
        elif current == 180:
            self.detect_walls(180, self.last_x - 2, self.last_y)
            self.maze.matrix[self.last_y][self.last_x - 1] = 'X'
            self.maze.matrix[self.last_y][self.last_x - 2] = 'X'
            self.last_x = xin + 27
        elif current == -90:
            self.detect_walls(-90, self.last_x, self.last_y + 2)
            self.maze.matrix[self.last_y + 1][self.last_x] = 'X'
            self.maze.matrix[self.last_y + 2][self.last_x] = 'X'
            self.last_y = -yin + 13

    # Função para detectar paredes
    def detect_walls(self, compass, x, y):
        value_to_detect = 1.2
        ir_sensors = self.measures.irSensor  # Sensores infravermelhos
        walls = []

        if ir_sensors[0] >= value_to_detect:  # Objeto à frente
            walls.append("front")
        if ir_sensors[1] >= value_to_detect:  # Objeto à esquerda
            walls.append("left")
        if ir_sensors[2] >= value_to_detect:  # Objeto à direita
            walls.append("right")

        if compass == 0:  # Norte
            if "front" in walls:
                self.maze.matrix[y][x + 1] = '|'
            if "left" in walls:
                self.maze.matrix[y - 1][x] = '-'
            if "right" in walls:
                self.maze.matrix[y + 1][x] = '-'
        elif compass == 90:  # Leste
            if "front" in walls:
                self.maze.matrix[y - 1][x] = '-'
            if "left" in walls:
                self.maze.matrix[y][x - 1] = '|'
            if "right" in walls:
                self.maze.matrix[y][x + 1] = '|'
        elif compass == 180:  # Sul
            if "front" in walls:
                self.maze.matrix[y][x - 1] = '|'
            if "left" in walls:
                self.maze.matrix[y + 1][x] = '-'
            if "right" in walls:
                self.maze.matrix[y - 1][x] = '-'
        elif compass == -90:  # Oeste
            if "front" in walls:
                self.maze.matrix[y + 1][x] = '-'
            if "left" in walls:
                self.maze.matrix[y][x + 1] = '|'
            if "right" in walls:
                self.maze.matrix[y][x - 1] = '|'

    # Função para rodar o robô
    def rotate(self, Kp, Kd, Ki, obj, retrot):
        if self.counterrot == 0:
            self.rot = 0.15
            self.integralrot = 0

        err = (obj - self.measures.compass) * math.pi / 180

        if self.rot != 0:
            diff = err / self.rot
        else:
            diff = 100
        self.integralrot += err
        self.rot = Kp * err + Kd * diff + Ki * self.integralrot
        self.lengthrot = err

        if not retrot:
            self.converter(0, self.rot)
            self.counter = 0
        self.counterrot += 1

        if -0.005 < self.lengthrot < 0.005:
            self.counterrot = 0
            return False
        return True
    
    # Função para corrigir a bússola
    def corrected_compass(self):
        current = self.measures.compass
        if -45 < current < 45:
            current = 0
        elif 45 < current < 135:
            current = 90
        elif 135 < current or current < -135:
            current = 180
        elif -100 < current < -80:
            current = -90

        return current

    # Função para comparar a bússola com a direção correta
    def compare_compass(self):
        current = self.measures.compass
        rotation_target = self.corrected_compass()
        difference = abs(rotation_target - current)
        return difference   

    # Função que encontra uma direção que esteja livre
    def find_free_direction(self):
        current = self.corrected_compass()

        if self.measures.irSensor[1] < 1:
            self.rotation_target = current + 90
        elif self.measures.irSensor[2] < 1:
            self.rotation_target = current - 90
        else:
            self.rotation_target = current + 180

        if self.rotation_target <= -180:
            self.rotation_target += 360
        if self.rotation_target > 180:
            self.rotation_target -= 360

    # Converte as coordenadas do GPS com base no ponto inicial
    def gpsConverter(self):
        if self.countergps == 0:
            self.xin = self.measures.x
            self.yin = self.measures.y
            self.countergps += 1
        self.measures.x -= self.xin
        self.measures.y -= self.yin
    
    def checkChangeCompass(self):
        if self.rotation_target == 180:
            self.South = True
        else:
            self.South = False

    # Regista a celula atual como visitada  
    def appendWalked(self):
        x = self.round_even(self.measures.x)
        y = self.round_even(self.measures.y)
        self.track_visited_cells.append((x, y))

    # Identifica celulas desconhecidas proximas
    def searchUnknown(self):
        x = self.round_even(self.measures.x)
        y = self.round_even(self.measures.y)
        current = radians(self.corrected_compass())
        entries = []

        if self.measures.irSensor[0] < 1:
            entries.append((x + round(cos(current)), y + round(sin(current))))
        if self.measures.irSensor[1] < 1:
            entries.append((x + round(cos(current + pi / 2)), y + round(sin(current + pi / 2))))
        if self.measures.irSensor[2] < 1:
            entries.append((x + round(cos(current - pi / 2)), y + round(sin(current - pi / 2))))

        for entry in entries:
            if entry not in self.unknown_cells and entry not in self.known_cells:
                self.unknown_cells.append(entry)

    # Atualiza listas de celulas conhecidas e desconhecidas
    def searchKnown(self):
        x = self.round_even(self.measures.x)
        y = self.round_even(self.measures.y)
        entry = (x, y)
        last_entry = self.track_visited_cells[-2]
        mid_entry = (int((last_entry[0] + entry[0]) / 2), (int((last_entry[1] + entry[1]) / 2)))
        equal = last_entry == mid_entry
        if entry in self.unknown_cells:
            self.unknown_cells.remove(entry)
        if mid_entry in self.unknown_cells and not self.first and not equal:
            self.unknown_cells.remove(mid_entry)
        if mid_entry not in self.known_cells and not equal:
            self.known_cells.append(mid_entry)
        if entry not in self.known_cells:
            self.known_cells.append(entry)
            return False
        else:
            return True

    def velEstimator(self, left_motor, right_motor):
        left_out = (left_motor + self.estimated_velocity[-1][0]) / 2
        right_out = (right_motor + self.estimated_velocity[-1][1]) / 2
        self.estimated_velocity.append((left_out, right_out))
        self.kinematics()

    # Calcula a nova posição global do robô
    def kinematics(self):
        wheel_velocity = np.transpose(np.asarray(self.estimated_velocity[-1]))
        velocity_matrix = np.asarray([[1 / 2, 1 / 2], [-1 / 2, 1 / 2]])
        global_velocity_matrix = np.asarray([[math.cos(math.radians(self.measures.compass)), 0],
                                             [math.sin(math.radians(self.measures.compass)), 0],
                                             [0, 1]])
        velocity = np.matmul(velocity_matrix, wheel_velocity)
        global_velocity = np.matmul(global_velocity_matrix, velocity)
        current_velocity = (global_velocity[0], global_velocity[1])
        last_pose = self.position_history[-1]
        current_pose = (last_pose[0] + current_velocity[0], last_pose[1] + current_velocity[1])
        corrected_pose = self.corrector(last_pose)
        if corrected_pose:
            current_pose = (corrected_pose[0] + current_velocity[0], corrected_pose[1] + current_velocity[1])
        self.position_history.append(current_pose)

    def distance(self, x):
        return 1 / x

    # Ajusta a posição do robô com base na proximidade de paredes
    def corrector(self, last_pose):
        current_pose = None
        wall = None
        direction = ""
        old_pose = last_pose
        center = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]
        robot_radius = 0.5
        distance_to_wall = 0.9
        difference_threshold = 2
        value_to_front = 1.5
        value_to_min_side = 0.4
        value_to_max_side = 2.0
        distance_threshold = 2.0

        if self.compare_compass() <= difference_threshold:
            if self.corrected_compass() == 0:
                if center >= value_to_front and back >= value_to_front:
                    wall = self.round_even(last_pose[0]) + distance_to_wall, last_pose[1]
                    current_pose = (wall[0] - self.distance((center + back)/2) - robot_radius, last_pose[1])
                    last_pose = current_pose
                    direction = 'Front '
                if left >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) + distance_to_wall
                    current_pose = (last_pose[0], wall[1] - self.distance(left) - robot_radius)
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) - distance_to_wall
                    current_pose = (last_pose[0], wall[1] + self.distance(right) + robot_radius)
                    direction = direction + 'Right'

            elif self.corrected_compass() == 90:
                if center >= value_to_front and back >= value_to_front:
                    wall = last_pose[0], self.round_even(last_pose[1]) + distance_to_wall
                    current_pose = (last_pose[0], wall[1] - self.distance((center + back)/2) - robot_radius)
                    last_pose = current_pose
                    direction = 'Front'
                if left >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) - distance_to_wall, last_pose[1]
                    current_pose = (wall[0] + self.distance(left) + robot_radius, last_pose[1])
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) + distance_to_wall, last_pose[1]
                    current_pose = (wall[0] - self.distance(right) - robot_radius, last_pose[1])
                    direction = direction + 'Right'

            elif self.corrected_compass() == 180:
                if center >= value_to_front and back >= value_to_front:
                    wall = self.round_even(last_pose[0]) - distance_to_wall, last_pose[1]
                    current_pose = (wall[0] + self.distance((center + back)/2) + robot_radius, last_pose[1])
                    last_pose = current_pose
                    direction = 'Front'
                if left >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) - distance_to_wall
                    current_pose = (last_pose[0], wall[1] + self.distance(left) + robot_radius)
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) + distance_to_wall
                    current_pose = (last_pose[0], wall[1] - self.distance(right) - robot_radius)
                    direction = direction + 'Right'

            elif self.corrected_compass() == -90:
                if center >= value_to_front and back >= value_to_front:
                    wall = last_pose[0], self.round_even(last_pose[1]) - distance_to_wall
                    current_pose = (last_pose[0], wall[1] + self.distance((center + back)/2) + robot_radius)
                    last_pose = current_pose
                    direction = 'Front'
                if left >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) + distance_to_wall, last_pose[1]
                    current_pose = (wall[0] - self.distance(left) - robot_radius, last_pose[1])
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) - distance_to_wall, last_pose[1]
                    current_pose = (wall[0] + self.distance(right) + robot_radius, last_pose[1])
                    direction = direction + 'Right'

            if current_pose:
                current_pose = (round(current_pose[0], 3), round(current_pose[1], 3))
                old_pose = (round(old_pose[0], 3), round(old_pose[1], 3))
                return current_pose

    def round_even(self, number):
        return round(number/2)*2

    def round_odd(self, number):
        difference = number - self.round_even(number)
        if difference >= 0:
            return round(number/2)*2 + 1
        else:
            return round(number/2)*2 - 1

    def converter(self, lin, rot):
        left_motor = lin - rot / 2
        right_motor = lin + rot / 2
        if left_motor > 0.15:
            left_motor = 0.15
        elif left_motor < -0.15:
            left_motor = -0.15
        if right_motor > 0.15:
            right_motor = 0.15
        elif right_motor < -0.15:
            right_motor = -0.15
        if not self.onRot:
            self.velEstimator(left_motor, right_motor)
        self.driveMotors(left_motor, right_motor)

class Lab():
    def __init__(self):
        self.matrix = [[' '] * 55]

        for m in range(26):
            self.matrix.insert(0, [' '] * 55)
        self.matrix[13][27] = 'I'

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
f = "default"

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = sys.argv[i + 1]
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        f = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 0.0], host)
    rob.f = f
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()