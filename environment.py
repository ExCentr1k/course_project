from dis import dis
import math
from turtle import heading, left
import numpy as np
import pygame

class buildEnvironment:
    def __init__(self):
        pygame.init()
        self.pointCloud = []
        self.externalMap = pygame.image.load('bahrain_33.png')
        #self.originalMap = pygame.image.load('bahrain_3.png')
        self.maph = self.externalMap.get_height()
        self.mapw = self.externalMap.get_width()
        self.robot = pygame.image.load('f1_car.png')
        self.MapWindowName = 'Navigation System'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0,0))
        self.trail_set = []
        self.threshold_max = 105
        self.threshold_min = 35    # sensetivity for boundaries

        self.black = (0, 0, 0)
        self.gray = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.Pink = (245, 90, 230)

    def Ad2pos(self, distance, angle, carPosition):
        x = distance * math.cos(angle) + carPosition[0]
        y = -distance * math.sin(angle) + carPosition[1]
        return(int(x), int(y))

    def show_sensorData(self, pointCloud_lidar):
        self.infomap = self.map.copy()
        for point in pointCloud_lidar[0]:
            self.infomap.set_at((int(point[0]), int(point[1])), (255, 128, 0))
        for point in pointCloud_lidar[1]:
            self.infomap.set_at((int(point[0]), int(point[1])), (0, 255, 255))

    def line_check(self, startpoint, endpoint):
        px = (startpoint[0] - endpoint[0]) ** 2
        py = (startpoint[1] - endpoint[1]) ** 2
        return math.sqrt(px + py)

    def draw_boundaries(self, pointCloud_lidar):
        #self.infomap = self.map.copy()
        for i in range(0, len(pointCloud_lidar[0]) - 1, 6):
            if self.threshold_min < self.line_check(pointCloud_lidar[0][i],
                    pointCloud_lidar[0][i+1]) < self.threshold_max:
                pygame.draw.line(self.map, (255, 128, 0), pointCloud_lidar[0][i],
                    pointCloud_lidar[0][i+1], 1)
        for i in range(0, len(pointCloud_lidar[1]) - 1, 6):
            if self.threshold_min < self.line_check(pointCloud_lidar[1][i],
                    pointCloud_lidar[1][i+1]) < self.threshold_max:
                pygame.draw.line(self.map, (0, 255, 255), pointCloud_lidar[1][i],
                    pointCloud_lidar[1][i+1], 1)
        

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center = (x, y))
        self.map.blit(rotated, rect)

    def trail(self, pos, color_flag):
        for i in range(0, len(self.trail_set) - 1):
            if color_flag == True:
                pygame.draw.line(self.map, self.Pink, (self.trail_set[i][0], self.trail_set[i][1]),
                    (self.trail_set[i+1][0], self.trail_set[i+1][1]), 2)
            else:
                pygame.draw.line(self.map, self.white, (self.trail_set[i][0], self.trail_set[i][1]),
                    (self.trail_set[i+1][0], self.trail_set[i+1][1]), 2)
        if self.trail_set.__sizeof__() > 13000:
              self.trail_set.pop(0)
        self.trail_set.append(pos)
    

    def draw_sensor_data(self, point_cloud, code):
        if code == 0:
            for point in point_cloud:
                pygame.draw.circle(self.map, self.Red, point, 3, 0)
        else:
            for point in point_cloud:
                pygame.draw.circle(self.map, self.Green, point, 3, 0)

#################################### robot

def distance_1(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, width):

        self.m2p = 3779.52  #meter to pixels

        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = math.pi

        self.vl = 0.01 * self.m2p
        self.vr = 0.01 * self.m2p

        self.maxspeed = 0.1 * self.m2p
        self.minspeed = 0.05 * self.m2p

        self.lapcount = 0

        self.min_obs_dist = 50
        self.count_down = 5

    def avoid_obstacles(self, point_cloud):
        closest_obs_r = (point_cloud[0], 500)
        dist = np.inf
        # for rightside point_cloud[0] - black
        if len(point_cloud[0]) >= 1:
            for point in point_cloud[0]:
                if dist > distance_1([self.x, self.y], point):
                        dist = distance_1([self.x, self.y], point)
                        closest_obs_r = (point, dist)

        # for leftside  point_cloud[1] - blue
        closest_obs_l = (point_cloud[1], 500)
        dist = np.inf        
        if len(point_cloud[1]) >= 1:
            for point in point_cloud[1]:
                if dist > distance_1([self.x, self.y], point):
                        dist = distance_1([self.x, self.y], point)
                        closest_obs_l = (point, dist)
       
        self.move_forward()
        if closest_obs_l[1] < self.min_obs_dist + (self.w / 2) + 7:# or len(point_cloud[1]) > len(point_cloud[0]):# and self.count_down > 0:
            self.turn_right()
        elif closest_obs_r[1] < self.min_obs_dist + (self.w / 2) + 7:# or len(point_cloud[0]) > len(point_cloud[1]):
            #self.count_down -= dt
            #print('right ' + str(closest_obs_r[1]))
            self.turn_left()     

    def turn_right(self):
        self.vr = self.minspeed / 4
        self.vl = self.minspeed 

    def turn_left(self):
        self.vr = self.minspeed
        self.vl = self.minspeed  / 4

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def kinematics(self, dt):
        self.x += ((self.vl + self.vr)/2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr)/2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt

        if self.heading > 2 * math.pi or self.heading < -2 * math.pi:
            self.heading = 0

    def lap(self, x, y):
        if x <= 853 and x >= 851  and y > 687 and y < 782:
            self.lapcount += 1
            print(x, y)
            print(self.lapcount)

class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map
        self.startfinishx = 0
        self.startfinishy = 0

    def sense_obstacles(self, x, y, heading):
        obstacles = [], []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 30, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles[0].append([x, y])
                        break
                    elif (color[0], color[1], color[2]) == (0, 0, 200):
                        obstacles[1].append([x, y])
                        break
                    #elif (color[0], color[1], color[2]) == (163, 73, 164):
                        #elf.startfinishx = x1
                        #self.startfinishy = y1
                        #break
        return obstacles