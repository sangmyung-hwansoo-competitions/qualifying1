#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 슈퍼카
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import rospy
from xycar_msgs.msg import xycar_motor
from math import cos, sin, sqrt, pow, atan2

from control_pid import PIDController
from control_longitudinal_planning import velocityPlanning
from control_pure_pursuit import PurePursuit
from model_vehicle import VehicleModel
from path_dubins import Dubins
from path_dubins import twopify, pify
#=============================================
# 모터 토픽을 발행할 것임을 선언
#=============================================
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
vehicle = None
cartesian_path = None
pid_controller = PIDController()
max_velocity = 50
velocity_planner = velocityPlanning(max_velocity/3.6, 0.15)
target_speeds = None
pure_pursuit = PurePursuit()
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
P_TARGET = (1105.75, 92.25, 5.497787143782138) # 주차라인 내부 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global vehicle, cartesian_path, pid_controller, velocity_planner, target_speeds, pure_pursuit

    kappa_ = 1./220
    dubins = Dubins()
    degree_alpha = 90

    print(syaw)

    syaw = np.deg2rad(syaw+degree_alpha)
    syaw = twopify(syaw)
    start_state = [sx, sy, syaw]

    print(f"start_state: {start_state}")

    vehicle = VehicleModel(length=80, width=64, rear_axle_ratio=0.6/2, x=sx, y=sy, yaw=syaw, v=0.0)

    gx_center, gy_center, gyaw = P_TARGET
    goal_state = [gx_center, gy_center, gyaw]

    print(f"goal_state: {goal_state}")

    cartesian_path, controls, dubins_path = dubins.plan(start_state, goal_state, kappa_)
    path_x, path_y, path_yaw = cartesian_path

    target_speeds = velocity_planner.curvedBaseVelocity(cartesian_path[:2], point_num=50)
    pure_pursuit.set_path(cartesian_path)

    return path_x, path_y

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global vehicle, cartesian_path, pid_controller, velocity_planner, target_speeds, pure_pursuit

    ## 225.0 + 90 = 315, 45 # 오른쪽 위 방향
    ## 180.0 + 90 = 270, 90 # 위 방향
    ## 135.0 + 90 = 225, 135 # 왼쪽 위 방향
    ## 135.0 + 90 = 225, 135 # 왼쪽 위 방향
    ## 225.0 + 90 = 315, 45 # 오른쪽 위 방향

    degree_alpha = 360
    # print(f"x: {x}, y: {y}, yaw: {yaw}, yaw: {twopify(np.deg2rad(-(yaw)+degree_alpha))}")

    if is_goal_reached(vehicle.x, vehicle.y, vehicle.yaw, P_TARGET):
        print("Goal reached.")
        if(vehicle.v > 0):
            drive(-5, 0)
        else:
            drive(0, 0)
        vehicle.update(vehicle.x, vehicle.y, vehicle.yaw, vehicle.v)
        return

    vehicle.update(x, y, twopify(np.deg2rad(-(yaw)+degree_alpha)), velocity)

    current_waypoint = pure_pursuit.get_current_waypoint(vehicle.x, vehicle.y)
    target_speed = target_speeds[current_waypoint]*3.6
    steer_angle = pure_pursuit.calc_pure_pursuit(current_waypoint, vehicle.x, vehicle.y, vehicle.yaw, vehicle.v, vehicle.length)
    throttle = pid_controller.control(target_speed, vehicle.v, dt)


    # pub_speed = vehicle.v + throttle * dt
    pub_angle = np.clip(np.rad2deg(steer_angle), -50, 50)
    throttle = np.clip(throttle, -20, 50)
    print(f"throttle: {throttle}, steer_angle: {pub_angle}")

    drive(pub_angle, throttle)


def is_goal_reached(current_x, current_y, current_yaw, P_TARGET, pos_threshold=3, yaw_threshold=0.3):
    goal_x, goal_y, goal_yaw = P_TARGET
    distance = sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
    yaw_diff = abs(twopify(current_yaw) - twopify(goal_yaw))
    return distance < pos_threshold and yaw_diff < yaw_threshold
