#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from math import *

class VehicleModel:
    """
    차량 모델 클래스

    차량의 속성 (길이, 너비, 축간 비율, 위치, 방향, 속도) 나타냄
    업데이트 및 좌표 계산, 차량 시각화 등의 기능을 제공
    """
    def __init__(self, length=50, width=30, axle_ratio=1/2, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """
        생성자 함수

        Args:
            length (float, optional): 차량 길이 (단위: m). Defaults to 50.
            width (float, optional): 차량 너비 (단위: m). Defaults to 30.
            axle_ratio (float, optional): 축간 비율 (전륜과 후륜 중심 간의 거리 / 차량 전체 길이). Defaults to 1/2.
            x (float, optional): 차량의 x 좌표 (단위: m). Defaults to 0.0.
            y (float, optional): 차량의 y 좌표 (단위: m). Defaults to 0.0.
            yaw (float, optional): 차량의 yaw 각도 (rad). Defaults to 0.0.
            v (float, optional): 차량의 속도 (단위: m/s). Defaults to 0.0.
        """
        self.length = length
        self.width = width
        self.axle_ratio = axle_ratio
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, x, y, yaw, v):
        """
        차량의 속성을 업데이트하는 함수

        Args:
            x (float): 새로운 x 좌표 (단위: m)
            y (float): 새로운 y 좌표 (단위: m)
            yaw (float): 새로운 yaw 각도 (rad)
            v (float): 새로운 속도 (단위: m/s)
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def get_front_coordinates(self):
        """
        차량의 정중심을 기준으로 전방 끝단의 좌표를 계산하여 반환하는 함수

        Returns:
            tuple: (front_x, front_y) - 차량 전방 끝단의 x, y 좌표
        """
        # 차량 길이와 축간 비율, yaw 각도를 이용하여 전방 끝단 좌표 계산
        front_x = self.x + (self.length * self.axle_ratio) * cos(self.yaw)
        front_y = self.y + (self.length * self.axle_ratio) * sin(self.yaw)
        return front_x, front_y

    def get_rear_coordinates(self):
        """
        차량의 정중심을 기준으로 후방 끝단의 좌표를 계산하여 반환하는 함수

        Returns:
            tuple: (rear_x, rear_y) - 차량 후방 끝단의 x, y 좌표
        """
        # 차량 길이와 축간 비율, yaw 각도를 이용하여 후방 끝단 좌표 계산
        rear_x = self.x - (self.length * self.axle_ratio) * cos(self.yaw)
        rear_y = self.y - (self.length * self.axle_ratio) * sin(self.yaw)
        return rear_x, rear_y

    def plot_vehicle(self, ax):
        """
        Matplotlib 라이브러리를 이용하여 차량을 시각화하는 함수

        Args:
            ax (matplotlib.axes._axes.Axes): 차량을 그릴 matplotlib ax 객체
        """
        x, y, yaw = self.x, self.y, self.yaw
        length, width = self.length, self.width
        front_x, front_y = self.get_front_coordinates() # 차량 전방 끝단 좌표
        rear_x, rear_y = self.get_rear_coordinates()    # 차량 후방 끝단 좌표

        # 차량의 네 모서리 (전방 상단, 전방 하단, 후방 하단, 후방 상단)의 상대적인 좌표 계산
        corners = np.array([
            [length/2, width/2],
            [length/2, -width/2],
            [-length/2, -width/2],
            [-length/2, width/2]
        ])

        # yaw 각도에 대한 회전 행렬
        R = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)]
        ])

        # corners 배열을 R행렬로 회전하고 [x, y] 벡터로 이동
        transformed_corners = np.dot(corners, R) + np.array([x, y])

        # 마지막 모서리 좌표와 처음 모서리 좌표를 연결해 사각형 모양 완성
        transformed_corners = np.vstack([transformed_corners, transformed_corners[0]])

        # Plot vehicle rectangle
        ax.plot(front_x, front_y, 'bo', label='car_front')  # 파란 원으로 전방 끝단 점 표시
        ax.plot(rear_x, rear_y, 'bo', label='car_rear')     # 파란 원으로 후방 끝단 점 표시
        ax.plot(transformed_corners[:, 0], transformed_corners[:, 1], 'k-')                 # 회전 및 변환된 모서리 좌표 연결하는 검은선 표시
        ax.fill(transformed_corners[:, 0], transformed_corners[:, 1], 'gray', alpha=0.5)    # 정의된 사각형을 회색으로 채움
