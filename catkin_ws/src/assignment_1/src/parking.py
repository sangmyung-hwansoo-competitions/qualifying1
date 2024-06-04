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
pid_controller = PIDController()                        # PID control을 이용해 제어
max_velocity = 50                                       # 최대 속도는 50
velocity_planner = velocityPlanning(max_velocity, 0.15) # 속도 계획
target_speeds = None                                    # 목표 속도는 우선 None
pure_pursuit = PurePursuit()                            # PurePursuit 알고리즘 기반 조향각도 계싼
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62)                                      # AR 태그의 위치
P_ENTRY = (1036, 162)                                # 주차라인 진입 시점의 좌표
P_END = (1129, 69)                                   # 주차라인 끝의 좌표
P_TARGET1 = (1059.25, 138.75, 5.497787143782138)     # 주차라인 내부 좌표, 1/4 지점
P_TARGET2 = (1082.5, 115.5, 5.497787143782138)       # 주차라인 내부 좌표, 2/4 지점
P_TARGET3 = (1105.75, 92.25, 5.497787143782138)      # 주차라인 내부 좌표, 3/4 지점
P_TARGET4 = (1117.375, 80.625, 5.497787143782138)    # 주차라인 내부 좌표, 3.5/4 지점

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.header.stamp = rospy.Time.now()
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
    """
    ## 경로 생성 함수

    Args:
        차량의 시작 위치 (sx, sy),
        시작 각도 (syaw), 
        최대 가속도 (max_acceleration),
        단위 시간 (dt)
    
    Returns:
        차량이 주행할 경로를 계산

    수행 과정:
    1. PID 컨트롤러 (global 변수) 참조
        - 코드 상에서 전역 변수인 `pid_controller` 를 사용
        - `pid_controller` 는 다른 함수에서도 사용되며, 차량의 속도 제어를 담당

    2. Dubins 파라미터 설정:
        - kappa: 곡선의 휘어짐 정도를 나타내는 파라미터
        - dubins: Dubins path planning 라이브러리 객체 생성
        - degree_alpha: 회전 각도 보정값 (인데 코드 내에서는 사용되지 않음)

    3. 차량 초기 상태 설정:
        - syaw 값을 라디안 각도로 변환하고 맵핑 공간에 맞게 조정 (twopify)
        - 시작 상태 (차량 위치 및 yaw) 저장

    4. 목표 지점 설정:
        - P_TARGETn 변수들은 주차 라인 내부의 여러 지점들의 좌표와 yaw 의미
        - 각 지점들은 차량의 경로상 중요한 지점으로 사용
        - P_END 는 주차 라인의 끝 지점의 좌표와 yaw를 의미

    5. 경로 생성:
        - waypoint: 시작 지점부터 목표 지점들, 그리고 종료 지점까지 연결된 위치 정보 리스트를 생성
        - dubins 라이브러리의 plan_waypoint 함수를 이용하여 waypoint 리스트를 기반으로 Dubins 경로를 생성
        - 생성된 경로 정보는 path_x, path_y, path_yaw 에 각각 차량의 x, y 좌표, yaw 값으로 저장

    6. 속도 계획:
        - velocity_planner 객체의 curvedBaseVelocity 함수를 통해 생성된 경로의 처음 두 지점을 기반으로 속도 프로파일을 생성
        - 생성된 속도 프로파일은 target_speeds 에 저장

    7. Pure Pursuit) 설정:
        - pure_pursuit 객체의 set_path 함수를 이용하여 생성된 경로 (cartesian_path) 를 설정

    8. 경로 반환:
        - 생성된 경로 정보 (path_x, path_y) 를 함수의 반환 값으로 제공
    """
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
    """
    ## 경로 추종 함수

    생성된 경로 (path_x, path_y) 를 따라 실제 차량을 주행

    수행 과정:

    1. PID controller (global 변수) 참조
        - 전역 변수인 `pid_controller` 를 사용

    2. 차량 상태 업데이트:
        - 현재 위치 (x, y), yaw, 속도 (velocity) 를 이용하여 차량 모델을 업데이트
        - 차량 모델은 차량의 위치, 방향, 속도 등을 나타내는 가상의 객체
        - `twopify` 함수를 사용해 yaw 값을 맵핑 공간에 맞게 조정

    3. 차량 앞쪽 위치 계산:
        - 차량 모델의 get_front_coordinates 함수를 사용
        -> 차량 앞쪽의 위치 (vehicle_front_x, vehicle_front_y) 를 계산

    4. 현재 웨이포인트 찾기:
        - pure_pursuit 객체의 get_current_waypoint 함수를 사용하여
        -> 차량 앞쪽 위치에 가장 가까운 웨이포인트 (current_waypoint) 를 찾음

    5. 목표 속도 계산:
        - target_speeds 리스트에서 현재 웨이포인트에 해당하는 목표 속도 (target_speed) 를 추출

    6. 조향각 계산:
        - pure_pursuit 객체의 calc_pure_pursuit 함수를 사용
        -> 목표 속도, 차량 앞쪽 위치, 현재 yaw, 속도, 차량 길이 등을 입력값으로 조향각 (steer_angle) 을 계산

    7. throttle 계산:
        - pid_controller 객체의 control 함수를 사용
        -> 목표 속도와 현재 속도, 단위 시간 (dt) 를 입력값으로 throttle 계산

    8. throttle 및 steering 제한:
        - 계산된 throttle과 steering을 각각 -50 ~ 50 범위 내로 제한

    9. 주차 완료 판단:
        - is_goal_reached 함수를 사용
        -> 현재 차량 위치, yaw, 목표 위치 (P_END) 를 비교하여 주차가 완료되었는지 판단
        - 주차가 완료되면 throttle과 steering을 0으로 설정

    10. drive 제어:
        - drive 함수를 사용하여 계산된 steering과 throttle를 실제 차량의 모터에 적용
    """

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
    """
    ## 주차 완료 판단 함수

    현재 차량 위치, yaw, 목표 위치를 입력값으로 하여 주차가 완료되었는지 판단

    Args:
    - current_x: 현재 차량의 x 좌표
    - current_y: 현재 차량의 y 좌표
    - current_yaw: 현재 차량의 yaw 값
    - P_TARGET: 주차 목표 위치 (x, y, yaw) 정보
    - pos_threshold: 위치 오차 허용 범위
    - yaw_threshold: yaw 값 오차 허용 범위

    Returns:
    - True: 주차가 완료
    - False: 주차가 완료되지 않음

    수행 과정:
    1. 목표 위치 정보 추출:
        - P_TARGET 변수에서 목표 위치 (goal_x, goal_y) 와 목표 yaw (goal_yaw) 값을 추출

    2. 위치 오차 계산:
        - 현재 차량 위치와 목표 위치 간의 거리를 `sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)` 로 계산

    3. yaw 값 오차 계산:
        - 현재 yaw 값과 목표 yaw 값의 차이를 계산하고 `abs(twopify(current_yaw) - twopify(goal_yaw))` 로 변환
        - `twopify` 함수는 yaw 값을 맵핑 공간에 맞게 조정

    4. 주차 완료 판단:
        - 계산된 위치 오차와 yaw 값 오차가 각각 허용 범위 (pos_threshold, yaw_threshold) 이하인 경우 
        -> 주차가 완료되었다고 판단

    5. 반환값:
        - 주차 완료 여부를 True 또는 False 로 반환
    """
    goal_x, goal_y = P_TARGET
    goal_yaw = 5.497787143782138
    distance = sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
    yaw_diff = abs(twopify(current_yaw) - twopify(goal_yaw))
    return distance < pos_threshold and yaw_diff < yaw_threshold
