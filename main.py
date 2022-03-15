from ast import Try
from turtle import heading, pos, position
import pygame
import math
import environment, sensors

env = environment.buildEnvironment()
#env.map.fill((128, 128, 128))
env.originalMap = env.map.copy()

laser = sensors.LaserSensor(170, uncertainty=(0.5, 0.01))

env.infomap = env.map.copy()

start = (880, 730)
robot = environment.Robot(start, 0.01 * 3779.52)

sensor_range = 300, math.radians(35)
ultra_sonic = environment.Ultrasonic(sensor_range, env.map)

dt = 0
last_time = pygame.time.get_ticks()
rect = pygame.Rect(863, 735, 0, 0).inflate(15, 130)
color = (255, 255, 255)
pygame.draw.rect(env.infomap, color, rect)

running = True
sensorOn = True
while running:

    #collide = rect.colliderect(env.robot.get_rect())
    collide = rect.collidepoint(robot.x, robot.y)
    if collide:
        robot.lap(robot.x, robot.y)
        color = (255, 0, 0)
        pygame.draw.rect(env.map, color, rect)
        pygame.display.flip()
    else:
        color = (255, 255, 255)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    if sensorOn:
        position = (robot.x, robot.y)
        laser.position = position
        sensor_data = laser.sense_obtacles()    # track boundaries

        robot.lap(robot.x, robot.y)
        if robot.lapcount >= 2:
            sensorOn = False
    
    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()

    env.map.blit(env.infomap, (0, 0))

    if sensorOn:
        env.show_sensorData(sensor_data)
        env.draw_boundaries(sensor_data)

    env.draw_boundaries(sensor_data)
    robot.kinematics(dt)
    env.draw_robot(robot.x, robot.y, robot.heading)
    env.trail((robot.x, robot.y), sensorOn)

    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading, sensorOn)
    robot.avoid_obstacles(point_cloud)
    env.draw_sensor_data(point_cloud[0], "left")
    env.draw_sensor_data(point_cloud[1], "right")

    pygame.display.update()