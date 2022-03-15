from doctest import OutputChecker
from tkinter.messagebox import RETRY
from turtle import distance
import numpy as np
import pygame
import math

def uncertainty_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covariance = np.diag(sigma ** 2)
    distance, angle = np.random.multivariate_normal(mean, covariance)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]

class LaserSensor:
    def __init__(self, Range, uncertainty):
        self.Range = Range
        self.speed = 4
        self.sigma = np.array([uncertainty[0], uncertainty[1]])
        self.position = (0,0)
        self.exmap = pygame.image.load('bahrain_33.png')
        self.W, self.H = pygame.display.get_surface().get_size()
        self.map = pygame.display.set_mode((self.W, self.H))
        self.map.blit(self.exmap, (0,0))
        self.data = []
        self.pointCloud_left = []
        self.pointCloud_right = []

    def distance(self, obstaclePosition):
        px = (obstaclePosition[0] - self.position[0]) ** 2
        py = (obstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(px + py)

    def Ad2pos(self, distance, angle, carPosition):
        x = distance * math.cos(angle) + carPosition[0]
        y = -distance * math.sin(angle) + carPosition[1]
        return(int(x), int(y))

    def sense_obtacles(self):
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0, math.pi * 2, 30, False):
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            for i in range(0, 50):
                u = i / 50
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.W and 0 < y < self.H:
                    color = self.map.get_at((x, y))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        point = self.Ad2pos(output[0], output[1], output[2])
                        if point not in self.pointCloud_left:
                            self.pointCloud_left.append(point)
                        #self.data.append(output)
                        break
                    if (color[0], color[1], color[2]) == (0, 0, 200):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        point = self.Ad2pos(output[0], output[1], output[2])
                        if point not in self.pointCloud_right:
                            self.pointCloud_right.append(point)
                        #self.data.append(output)
                        break
        return [self.pointCloud_right, self.pointCloud_left]
