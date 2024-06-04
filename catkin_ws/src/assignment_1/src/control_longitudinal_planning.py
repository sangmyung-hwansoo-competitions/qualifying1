#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt

class velocityPlanning:
    """
    이 클래스는 주어진 경로를 따라 이동하는 차량의 속도 계획을 수행

    1. 초기 속도 설정: 최대 속도의 40%로 설정
    2. 곡률 기반 속도 계획: 각 지점의 곡률을 계산하고, 곡률에 따라 최대 속도를 조절
    3. 감속 구간 설정: 목적지에 가까워질수록 속도를 점차 감소
    4. 정지 구간 설정: 목적지에 도달하면 속도를 0으로 설정

    Args:
        max_speed (float): 차량의 최대 속도
        road_friction (float): 도로의 마찰 계수
    """
    def __init__ (self, max_speed, road_friciton):
        """
        생성자 함수

        Args:
            max_speed (float): 차량의 최대 속도
            road_friction (float): 도로의 마찰 계수
        """
        self.max_speed = max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        """
        주어진 경로를 따라 이동하는 차량의 속도 계획을 수행

        Args:
            global_path (list of lists): 경로의 x, y 좌표를 저장하는 2d list
            point_num (int): 곡률 계산에 사용할 주변 data point 개수

        Returns:
            list: 각 지점의 계획된 속도를 저장하는 list
        """
        out_vel_plan = []

        # 1. 초기 속도 설정
        for i in range(0, point_num):
            out_vel_plan.append(self.max_speed * 0.4)

        # 2. 곡률 기반 속도 계획
        for i in range(point_num, len(gloabl_path[0])- point_num):
            x_list = []
            y_list = []

            # 주변 data point 저장
            for box in range(-point_num, point_num):
                x = gloabl_path[0][i+box]
                y = gloabl_path[1][i+box]
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            # 도로의 곡률 계산
            # 곡률 계산 수식 그대로
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.pinv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            # 곡률 기반 최대 속도 계획
            v_max = sqrt(r*9.8*self.road_friction)

            # # 감속 계수 (곡률이 더 클 때 더 많이 감속)
            # if r < 5:       # 곡률 반경이 작을 때 더 많이 감속
            #     deceleration_factor = 0.8
            # elif r < 10:    # 중간 정도의 곡률 반경
            #     deceleration_factor = 0.85
            # elif r < 20:    # 중간 정도의 곡률 반경
            #     deceleration_factor = 0.9
            # else:           # 곡률 반경이 클 때 덜 감속
            #     deceleration_factor = 1.0  
            
            # # 반영
            # v_max *= deceleration_factor

            # 최대 속도 제한
            if v_max > self.max_speed:
                v_max = self.max_speed

            out_vel_plan.append(v_max)

        # 3. 감속 구간 설정
        # 목적지에 가까워질 수록 속도 점차 감소
        # point_num ~ 마지막 50개 point에 대해 점점 최대 속도의 20%씩 감소
        for i in range(len(gloabl_path[0])-point_num, len(gloabl_path[0])-50):
            out_vel_plan.append(self.max_speed * 0.2)

        # 4. 정지 구간 설정
        # 마지막 50개 point ~ 목적지까지 속도를 0
        for i in range(len(gloabl_path[0])-50, len(gloabl_path[0])):
            out_vel_plan.append(0)

        return out_vel_plan
