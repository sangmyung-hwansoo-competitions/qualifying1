#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2

class PurePursuit:
    """
    순수 추적 (Pure Pursuit) 알고리즘을 구현하는 클래스

    자동차가 주어진 경로를 따라가도록 하는 제어 방식 중 하나
    로봇의 현재 위치와 바라보는 방향, 목표 지점 (Look-Ahead Point) 사이의 관계를 이용 
    -> 조향 각도 (Steering Angle) 를 계산

    """
    def __init__(self):
        """
        생성자 함수

        알고리즘에 사용될 변수들을 초기화

        Args:
            없음
        """
        # Parmeter
        # Look-Ahead Distance (LFd) 관련 파라미터
        self.lfd = 0.0                    # 현재 Look-Ahead Distance 값
        self.min_lfd = 3.0                # 최소 Look-Ahead Distance 값
        self.max_lfd = 30.0               # 최대 Look-Ahead Distance 값
        self.lfd_gain = 0.6               # 속도 비례 Look-Ahead Distance 계산 gain 

        # 경로 및 목표 지점 관련 변수
        self.path = None                    # 경로 데이터 (2d List -> [x좌표 List, y좌표 List])
        self.forward_point = None           # 목표 지점 (Look-Ahead Point) 좌표
        self.is_look_forward_point = False  # 목표 지점 설정 여부 플래그

    def set_path(self, path):
        """
        알고리즘에 사용할 경로 데이터를 설정하는 함수

        Args:
            path (List): 경로 데이터 (2d List: [x좌표 List, y좌표 List])
        """
        self.path = path

    def get_current_waypoint(self, x, y):
        """
        자동차가 현재 위치에서 가장 가까운 경로상의 지점 인덱스를 반환하는 함수

        Args:
            x (float): 자동차의 x 좌표
            y (float): 자동차의 y 좌표

        Returns:
            int: 자동차가 현재 위치에서 가장 가까운 경로상의 지점 인덱스
        """
        min_dist = float('inf')         # 가장 가까운 지점까지의 거리 (초기값: 무한대)
        current_waypoint = -1           # 가장 가까운 지점 인덱스 (초기값: -1)

        for i in range(len(self.path[0])):
            dx = x - self.path[0][i]                # x 좌표 차이
            dy = y - self.path[1][i]                # y 좌표 차이
            dist = sqrt(pow(dx, 2) + pow(dy, 2))    # 거리 계산
            
            # 더 가까운 지점 발견하면 업데이트
            if min_dist > dist:                     
                min_dist = dist
                current_waypoint = i

        return current_waypoint

    def calc_pure_pursuit(self, current_waypoint, x, y, yaw, v, length):
        """
        알고리즘을 이용하여 조향 각도를 계산하는 함수

        Args:
            current_waypoint (int): 자동치의 현재 위치에서 가장 가까운 경로상의 지점 인덱스
            x (float): 자동차의 x 좌표
            y (float): 자동차의 y 좌표
            yaw (float): 자동차의 yaw 각도 (rad)
            v (float): 자동차의 속도
            length (float): 자동차의 차체 길이

        Returns:
            float: 계산된 조향 각도 (rad)
        """
        # 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (v) * self.lfd_gain
        
        # Look-Ahead Distance 제한값 적용
        if self.lfd < self.min_lfd :
            self.lfd=self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd=self.max_lfd

        # 목표 지점 설정 여부 초기화
        self.is_look_forward_point= False

        # 자동차 위치 변환 (좌표계 변환)
        translation = [x, y]

        # 좌표 변환 행렬 생성
        trans_matrix = np.array([
                [cos(yaw), -sin(yaw), translation[0]],
                [sin(yaw), cos(yaw), translation[1]],
                [0       , 0       , 1]])

        # 역행렬 계산
        det_trans_matrix = np.linalg.inv(trans_matrix)

        # 목표 지점 탐색
        for i in range(current_waypoint, len(self.path[0])):
            path_point = [self.path[0][i], self.path[1][i], 1]  # 경로상의 지점 좌표
            local_path_point = det_trans_matrix.dot(path_point) # 자동차 기준 좌표 변환

            # Look-Ahead Distance 조건 만족 여부 확인
            if local_path_point[0] > 0:  # 자동차 앞쪽에 있는 지점인 경우
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        # Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0]) # 목표 지점 방향 각도
        steering = atan2((2 * (length) * sin(theta)), self.lfd) # 조향 각도 계산

        return steering

'''
# 나중을 위한 예외처리 코드
if self.is_look_forward_point:
    theta = atan2(local_path_point[1], local_path_point[0])  # 목표 지점 방향 각도
    steering = atan2((2 * length * sin(theta)), self.lfd)  # 조향 각도 계산
    return steering
else:
    return 0  # 목표 지점 설정 불가능 시 0 반환
'''