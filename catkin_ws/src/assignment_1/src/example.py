#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from control_pid import PIDController
from control_longitudinal_planning import velocityPlanning
from control_pure_pursuit import PurePursuit
from model_vehicle import VehicleModel
from path_dubins import Dubins
from path_dubins import twopify, pify

from math import cos, sin, sqrt, pow, atan2
import numpy as np
import matplotlib.pyplot as plt

## 시작 지점, 왼쪽 위부터 반시계방향
# [sx, sy, syaw, max_acceleration, dt]
# 100.0, 350.0, 225.0, 1000.0, 0.016
# 100.0, 750.0, 180.0, 1000.0, 0.016
# 500.0, 700.0, 135.0, 1000.0, 0.016
# 900.0, 700.0, 135.0, 1000.0, 0.016
# 1100.0, 300.0, 225.0, 1000.0, 0.016

## 실행 예시

def is_goal_reached(current_x, current_y, current_yaw, goal_x, goal_y, goal_yaw, pos_threshold=3, yaw_threshold=0.3):
    distance = sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
    yaw_diff = abs(twopify(current_yaw) - twopify(goal_yaw))
    return distance < pos_threshold and yaw_diff < yaw_threshold

def straight_line_path(start, end, discretization=0.1):
    x1, y1, yaw1 = start
    x2, y2, yaw2 = end

    # 두 점 사이의 거리 계산
    distance = sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # 필요한 분할 횟수 계산
    num_points = int(np.ceil(distance / discretization))

    # 각 분할 지점의 좌표를 저장할 리스트 초기화
    xs, ys, yaws = [], [], []

    # x, y 좌표와 yaw 값의 분할 간격 계산
    delta_x = (x2 - x1) / num_points
    delta_y = (y2 - y1) / num_points
    delta_yaw = (yaw2 - yaw1) / num_points

    # 분할 지점의 좌표 계산 및 리스트에 추가
    for i in range(num_points + 1):
        x = x1 + i * delta_x
        y = y1 + i * delta_y
        yaw = yaw1 + i * delta_yaw
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)

    return xs, ys, yaws

if __name__ == '__main__':
    kappa_ = 1./220
    dubins = Dubins()

    degree_alpha = 90

    sx1, sy1, syaw = 1100.0, 300.0, 225.0
    syaw = np.deg2rad(syaw+degree_alpha)
    syaw = twopify(syaw)
    start_state1 = [sx1, sy1, syaw]

    vehicle = VehicleModel(length=128, width=64,rear_axle_ratio=0.6/2, x=sx1, y=sy1, yaw=syaw, v=0.0)
    sx2, sy2 = vehicle.get_rear_coordinates()
    start_state2 = [sx2, sy2, syaw]

    gx1, gy1 = 1036, 162    # 주차라인 진입 시점의 좌표
    gx2, gy2 = 1129, 69     # 주차라인 끝의 좌표
    gx_center = (gx1 + gx2) / 2
    gy_center = (gy1 + gy2) / 2
    gyaw = np.arctan2(gy2 - gy1, gx2 - gx1) # 진입점과 끝점의 각도 차이
    gyaw = twopify(gyaw)
    goal_state = [gx_center, gy_center, gyaw]

    cartesian_path1, controls1, dubins_path1 = dubins.plan(start_state1, goal_state, kappa_)
    path_x1, path_y1, path_yaw1 = cartesian_path1

    cartesian_path2, controls2, dubins_path2 = dubins.plan(start_state2, goal_state, kappa_)
    path_x2, path_y2, path_yaw2 = cartesian_path2

    pid_controller = PIDController()
    max_velocity = 50
    velocity_planner = velocityPlanning(max_velocity/3.6, 0.15)
    target_speeds = velocity_planner.curvedBaseVelocity(cartesian_path1[:2], point_num=50)
    pure_pursuit = PurePursuit()
    pure_pursuit.set_path(cartesian_path1)


    dt = 0.1
    fig, ax = plt.subplots()
    current_speed = vehicle.v
    current_x, current_y, current_yaw = vehicle.x, vehicle.y, vehicle.yaw

    result_path_x = []
    result_path_y = []

    for i in range(len(cartesian_path1[0])):
        result_path_x.append(vehicle.x)
        result_path_y.append(vehicle.y)

        if is_goal_reached(vehicle.x, vehicle.y, vehicle.yaw, gx_center, gy_center, gyaw):
            print("Goal reached.")
            vehicle.update(vehicle.x, vehicle.y, vehicle.yaw, 0.0)
            break

        current_waypoint = pure_pursuit.get_current_waypoint(vehicle.x, vehicle.y)
        target_speed = target_speeds[current_waypoint]*3.6
        steer_angle = pure_pursuit.calc_pure_pursuit(current_waypoint, vehicle.x, vehicle.y, vehicle.yaw, vehicle.v, vehicle.length)
        throttle = pid_controller.control(target_speed, vehicle.v, dt)
        print(f"throttle: {throttle}, steer_angle: {steer_angle}")

        current_speed += throttle * dt
        current_x += current_speed * cos(current_yaw) * dt
        current_y += current_speed * sin(current_yaw) * dt
        current_yaw += steer_angle * dt
        vehicle.update(current_x, current_y, current_yaw, current_speed)

        ax.clear()
        ax.plot(sx1, sy1, 'ro', label='start')
        ax.plot(gx1, gy1, 'rx', label='goal_startline')
        ax.plot(gx2, gy2, 'rx', label='goal_endline')
        ax.plot(path_x1, path_y1, 'g-')
        ax.plot(path_x2, path_y2, 'b-')
        ax.plot(result_path_x, result_path_y, 'r-', label='result')

        ax.arrow(sx1, sy1, 40 * np.cos(syaw), 40 * np.sin(syaw), head_width=10, head_length=10, fc='r', ec='r')
        ax.arrow(gx1, gy1, 40 * np.cos(gyaw), 40 * np.sin(gyaw), head_width=10, head_length=10, fc='r', ec='r')

        vehicle.plot_vehicle(ax)

        ax.axis("equal")
        ax.set_xlim(0, 1600)
        ax.set_ylim(-200, 1000)
        ax.invert_yaxis()
        ax.legend()
        plt.pause(0.1)

    plt.show()
