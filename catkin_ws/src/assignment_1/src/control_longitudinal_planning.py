#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from math import sqrt

class velocityPlanning:
    def __init__ (self, max_speed, road_friciton):
        self.max_speed = max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.max_speed * 0.45)

        for i in range(point_num, len(gloabl_path[0])- point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path[0][i+box]
                y = gloabl_path[1][i+box]
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            # 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.pinv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            # 곡률 기반 속도 계획
            v_max = sqrt(r*9.8*self.road_friction)

            # 감속 계수 (곡률이 더 클 때 더 많이 감속)
            # deceleration_factor = 0.7  # 감속 조절 0.65 -> 0.8 -> 0.65
            # v_max *= deceleration_factor

            if v_max > self.max_speed:
                v_max = self.max_speed

            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path[0]) - point_num, len(gloabl_path[0])-30):
            out_vel_plan.append(self.max_speed)

        for i in range(len(gloabl_path[0]) - 30, len(gloabl_path[0])):
            out_vel_plan.append(0)

        return out_vel_plan
