#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2

class PurePursuit:
    def __init__(self):
        # Parmeter
        self.lfd = 0.0
        self.min_lfd = 3.0
        self.max_lfd = 30
        self.lfd_gain = 0.6

        self.path = None
        self.forward_point = None
        self.is_look_forward_point = False

    def set_path(self, path):
        self.path = path

    def get_current_waypoint(self, x, y):
        min_dist = float('inf')
        current_waypoint = -1
        for i in range(len(self.path[0])):
            dx = x - self.path[0][i]
            dy = y - self.path[1][i]
            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_pure_pursuit(self, current_waypoint, x, y, yaw, v, length):
        # 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (v) * self.lfd_gain

        if self.lfd < self.min_lfd :
            self.lfd=self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd=self.max_lfd

        self.is_look_forward_point= False

        translation = [x, y]

        # 좌표 변환 행렬 생성
        trans_matrix = np.array([
                [cos(yaw), -sin(yaw), translation[0]],
                [sin(yaw), cos(yaw), translation[1]],
                [0       , 0       , 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for i in range(current_waypoint, len(self.path[0])):
            path_point = [self.path[0][i], self.path[1][i], 1]
            local_path_point = det_trans_matrix.dot(path_point)

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        # Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * (length) * sin(theta)), self.lfd)

        return steering
