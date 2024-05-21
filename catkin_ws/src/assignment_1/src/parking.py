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
velocity_planner = velocityPlanning(max_velocity, 0.15)
target_speeds = None
pure_pursuit = PurePursuit()
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
P_TARGET1 = (1059.25, 138.75, 5.497787143782138) # 주차라인 내부 좌표, 1/4 지점
P_TARGET2 = (1082.5, 115.5, 5.497787143782138) # 주차라인 내부 좌표, 2/4 지점
P_TARGET3 = (1105.75, 92.25, 5.497787143782138) # 주차라인 내부 좌표, 3/4 지점
P_TARGET4 = (1117.375, 80.625, 5.497787143782138) # 주차라인 내부 좌표, 3.5/4 지점

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

    syaw = np.deg2rad(syaw+degree_alpha)
    syaw = twopify(syaw)
    start_state = [sx, sy, syaw]

    print(f"start_state: {start_state}")

    vehicle = VehicleModel(length=128, width=64, x=sx, y=sy, yaw=syaw, v=0.0)
    vehicle_front_x, vehicle_front_y = vehicle.get_front_coordinates()
    start_state_front = [vehicle_front_x, vehicle_front_y, syaw]

    gx1, gy1, gyaw = P_TARGET1
    goal_state1 = [gx1, gy1, gyaw]

    gx2, gy2, _ = P_TARGET2
    goal_state2 = [gx2, gy2, gyaw]

    gx3, gy3, _ = P_TARGET3
    goal_state3 = [gx3, gy3, gyaw]

    gx4, gy4, _ = P_TARGET4
    goal_state4 = [gx4, gy4, gyaw]

    gx5, gy5 = P_END
    goal_state5 = [gx5, gy5, gyaw]

    waypoint = [start_state_front, goal_state1, goal_state2, goal_state3, goal_state4, goal_state5]

    print(f"waypoint: {waypoint}")

    cartesian_path, controls, dubins_path = dubins.plan_waypoint(waypoint, kappa_)
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

    vehicle.update(x, y, twopify(np.deg2rad(-(yaw)+degree_alpha)), velocity)
    vehicle_front_x, vehicle_front_y = vehicle.get_front_coordinates()

    current_waypoint = pure_pursuit.get_current_waypoint(vehicle_front_x, vehicle_front_y)
    target_speed = target_speeds[current_waypoint]
    steer_angle = pure_pursuit.calc_pure_pursuit(current_waypoint, vehicle_front_x, vehicle_front_y, vehicle.yaw, vehicle.v, vehicle.length)
    throttle = pid_controller.control(target_speed, vehicle.v, dt)

    throttle = vehicle.v + throttle * dt

    # pub_speed = vehicle.v + throttle * dt
    pub_angle = np.clip(np.rad2deg(steer_angle), -50, 50)
    throttle = np.clip(throttle, -50, 50)

    if is_goal_reached(vehicle_front_x, vehicle_front_y, vehicle.yaw, P_END):
        throttle = 0
        pub_angle = 0

    drive(pub_angle, throttle)


# 0.18 = 10.도
def is_goal_reached(current_x, current_y, current_yaw, P_TARGET, pos_threshold=3, yaw_threshold=0.18):
    goal_x, goal_y = P_TARGET
    goal_yaw = 5.497787143782138
    distance = sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
    yaw_diff = abs(twopify(current_yaw) - twopify(goal_yaw))
    return distance < pos_threshold and yaw_diff < yaw_threshold
