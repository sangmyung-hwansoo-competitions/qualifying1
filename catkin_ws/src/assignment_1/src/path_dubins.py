#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

def twopify(alpha):
    """
    주어진 각도(alpha)를 0과 2π 사이의 값으로 조정

    Args:
        alpha: 조정할 각도 (라디안)

    Returns:
        0과 2π 사이의 조정된 각도
    """
    return alpha - np.pi * 2 * np.floor(alpha / (np.pi * 2))

def pify(alpha):
    """
    주어진 각도(alpha)를 -π와 π 사이의 값으로 조정

    Args:
        alpha: 조정할 각도 (라디안)

    Returns:
        -π와 π 사이의 조정된 각도
    """
    v = np.fmod(alpha, 2*np.pi)
    if (v < - np.pi):
        v += 2 * np.pi
    else:
        v -= 2 * np.pi
    return v


class DubinsPath(object):
    """
    Dubins 경로를 나타내는 클래스입니다.

    Attributes:
        t: 첫 번째 원형 경로 길이
        p: 직선 경로 길이
        q: 두 번째 원형 경로 길이
        length_: 각 경로 세그먼트의 길이 목록
        type: 각 경로 세그먼트의 유형 목록 (L, S, R)
        controls: 각 경로 세그먼트에 대한 Dubins 제어 객체 (나중에 설정됨)
    """
    def __init__(self, t=0, p=1e10, q=0, type=None):
        self.t = t
        self.p = p
        self.q = q
        self.length_ = [t, p, q]
        self.type = type
        self.controls = None

    def length(self):
        """
        Dubins 경로의 총 길이를 계산

        Returns:
            각 경로 세그먼트 길이의 합인 총 길이
        """
        return self.t + self.p + self.q


class DubinsControl(object):
    """
    경로 세그먼트에 대한 제어 정보를 나타내는 클래스

    Attributes:
        delta_s: 경로 세그먼트를 따라 이동한 거리
        kappa: 원형 경로 세그먼트의 곡률
    """
    def __init__(self):
        self.delta_s = 0.0
        self.kappa = 0.0


class Dubins(object):
    """
    Dubins 경로 계획을 위한 클래스입니다.

    Attributes:
        constant: 허용 오차를 확인하는 데 사용되는 상수를 저장하는 딕셔너리
        dubinsLSL: 시작 각도와 목표 각도가 모두 왼쪽 방향인 경우 LSL(Left-Straight-Left) Dubins 경로를 계산
        dubinsRSR: 시작 각도와 목표 각도가 모두 오른쪽 방향인 경우 RSR(Right-Straight-Right) Dubins 경로를 계산
        dubinsRSL: 시작 각도가 오른쪽, 목표 각도가 왼쪽인 경우 RSL(Right-Straight-Left) Dubins 경로를 계산
        dubinsLSR: 시작 각도가 왼쪽, 목표 각도가 오른쪽인 경우 LSR(Left-Straight-Right) Dubins 경로를 계산
        dubinsRLR: 시작 각도와 목표 각도가 서로 반대인 경우 RLR(Right-Left-Right) Dubins 경로를 계산
        dubinsLRL: 시작 각도와 목표 각도가 서로 반대인 경우 LRL(Left-Right-Left) Dubins 경로를 계산
        get_best_dubins_path: 가능한 모든 경로 유형(LSL, RSR 등) 중에서 가장 짧은 Dubins 경로를 찾음
        dubins_path_to_controls: Dubins 경로 객체를 기반으로 각 경로 세그먼트를 따라 이동하는 제어 입력값을 계산
        controls_to_cartesian_path: 제어 입력값을 사용하여 실제 차량의 이동 경로 (x, y, yaw) 좌표를 계산
        plan: 시작 상태와 목표 상태, 곡률(kappa) 값을 입력받아 Dubins 경로를 생성
        plan_waypoint: 여러 개의 지점(waypoint) 목록을 입력받아 각 지점 사이를 연결하는 Dubins 경로를 생성
    """
    def __init__(self):
        '''
        클래스 초기화
        허용 오차를 나타내는 상수를 저장
        '''
        self.constant = {
            "dubins_zero": -1e-9,
            "dubins_eps": 1e-6
        }

    def dubinsLSL(self, d, alpha, beta):
        """
        시작 각도와 목표 각도가 모두 왼쪽 방향인 경우 LSL(Left-Straight-Left) Dubins 경로를 계산

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            계산된 LSL Dubins 경로 (DubinsPath 객체) 또는 경로가 존재하지 않는 경우 None
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = 2 + d * d - 2 * (ca * cb + sa * sb - d * (sa - sb))
        if (tmp >= self.constant["dubins_zero"]):
            theta = np.arctan2(cb - ca, d + sa - sb)
            t = twopify(-alpha + theta)
            p = np.sqrt(np.amax([tmp, 0]))
            q = twopify(beta - theta)

            return DubinsPath(t, p, q, ["L", "S", "L"])

        else:
            return None

    def dubinsRSR(self, d, alpha, beta):
        """
        시작 각도와 목표 각도가 모두 오른쪽 방향인 경우 RSR(Right-Straight-Right) Dubins 경로를 계산

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            계산된 RSR Dubins 경로 (DubinsPath 객체) 또는 경로가 존재하지 않는 경우 None
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa))
        if (tmp >= self.constant["dubins_zero"]):
            theta = np.arctan2(ca - cb, d - sa + sb)
            t = twopify(alpha - theta)
            p = np.sqrt(np.amax([tmp, 0]))
            q = twopify(-beta + theta)
            return DubinsPath(t, p, q, ["R", "S", "R"])
        else:
            return None

    def dubinsRSL(self, d, alpha, beta):
        """
        시작 각도가 오른쪽, 목표 각도가 왼쪽인 경우 RSL(Right-Straight-Left) Dubins 경로를 계산

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            계산된 RSL Dubins 경로 (DubinsPath 객체) 또는 경로가 존재하지 않는 경우 None
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb))
        if (tmp >= self.constant["dubins_zero"]):
            p = np.sqrt(np.amax([tmp, 0]))
            theta = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2., p)
            t = twopify(alpha - theta)
            q = twopify(beta - theta)
            return DubinsPath(t, p, q, ["R", "S", "L"])
        else:
            return None

    def dubinsLSR(self, d, alpha, beta):
        """
        시작 각도가 왼쪽, 목표 각도가 오른쪽인 경우 LSR(Left-Straight-Right) Dubins 경로를 계산

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            계산된 LSR Dubins 경로 (DubinsPath 객체) 또는 경로가 존재하지 않는 경우 None
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb))
        if (tmp >= self.constant["dubins_zero"]):
            p = np.sqrt(np.amax([tmp, 0]))
            theta = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2., p)
            t = twopify(-alpha + theta)
            q = twopify(-beta + theta)
            return DubinsPath(t, p, q, ["L", "S", "R"])
        else:
            return None

    def dubinsRLR(self, d, alpha, beta):
        """
        시작 각도와 목표 각도가 서로 반대인 경우 RLR(Right-Left-Right) Dubins 경로를 계산

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            계산된 RLR Dubins 경로 (DubinsPath 객체) 또는 경로가 존재하지 않는 경우 None
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)))
        if (np.abs(tmp) < 1.):
            p = 2 * np.pi - np.arccos(tmp)
            theta = np.arctan2(ca - cb, d - sa + sb)
            t = twopify(alpha - theta + .5 * p)
            q = twopify(alpha - beta - t + p)
            return DubinsPath(t, p, q, ["R", "L", "R"])
        else:
            return None

    def dubinsLRL(self, d, alpha, beta):
        """
        시작 각도와 목표 각도가 서로 반대인 경우 LRL(Left-Right-Left) Dubins 경로를 계산

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            계산된 LRL Dubins 경로 (DubinsPath 객체) 또는 경로가 존재하지 않는 경우 None
        """
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)))
        if (np.abs(tmp) < 1.):
            p = 2 * np.pi - np.arccos(tmp)
            theta = np.arctan2(-ca + cb, d + sa - sb)
            t = twopify(-alpha + theta + .5 * p)
            q = twopify(beta - alpha - t + p)
            return DubinsPath(t, p, q, ["L", "R", "L"])
        else:
            return None

    def get_best_dubins_path(self, d, alpha, beta):
        """
        가능한 모든 경로 유형(LSL, RSR 등) 중에서 가장 짧은 Dubins 경로를 찾음.

        Args:
            d: 시작 위치와 목표 위치 사이의 거리
            alpha: 시작 각도 (라디안)
            beta: 목표 각도 (라디안)

        Returns:
            가장 짧은 Dubins 경로를 나타내는 DubinsPath 객체 또는 None (경로가 존재하지 않는 경우)
        """
        dubins_functions = [
            self.dubinsLSL, self.dubinsRSR, self.dubinsRSL,
            self.dubinsLSR, self.dubinsRLR, self.dubinsLRL
        ]

        min_length = 1e10
        path = None
        for dubins_function in dubins_functions:
            tmp_path = dubins_function(d, alpha, beta)
            if tmp_path is not None:
                if (tmp_path.length() < min_length):
                    min_length = tmp_path.length()
                    path = tmp_path
        return path

    def dubins_path_to_controls(self, dubins_path, kappa):
        """
        Dubins 경로 객체를 기반으로 각 경로 세그먼트를 따라 이동하는 제어 입력값을 계산

        Args:
            dubins_path: Dubins 경로를 나타내는 DubinsPath 객체
            kappa: 차량의 곡률 (1/m)

        Returns:
            각 경로 세그먼트에 대한 제어 입력값을 담은 DubinsControl 객체 목록
        """
        controls = []
        kappa_inv = 1.0/kappa

        if dubins_path is not None:
            for i in range(3):
                control = DubinsControl()
                type = dubins_path.type[i]
                length = dubins_path.length_[i]
                delta_s = kappa_inv * length

                control.delta_s = delta_s
                if (type == "L"):
                    control.kappa = kappa

                if (type == "S"):
                    control.kappa = 0

                if (type == "R"):
                    control.kappa = -kappa

                controls.append(control)
            return controls

        else:
            return None

    def controls_to_cartesian_path(self, controls, state1, discretization=0.1):
        """
        제어 입력값을 사용하여 실제 차량의 이동 경로 (x, y, yaw) 좌표를 계산

        Args:
            controls: 각 경로 세그먼트에 대한 제어 입력값을 담은 DubinsControl 객체 목록
            state1: 시작 상태 (x, y, yaw)

        Returns:
            차량의 이동 경로를 나타내는 x, y, yaw 좌표 목록
        """
        if controls is None:
            return None

        x, y, yaw = state1
        xs, ys, yaws = [], [], []

        for control in controls:
            delta_s = control.delta_s
            abs_delta_s = np.abs(delta_s)
            kappa = control.kappa

            s_seg = 0
            integration_step = 0.0
            for j in range(int(np.ceil(abs_delta_s / discretization))):
                s_seg += discretization
                if (s_seg > abs_delta_s):
                    integration_step = discretization - (s_seg - abs_delta_s)
                    s_seg = abs_delta_s
                else:
                    integration_step = discretization

                if np.abs(kappa) > 0.0001:
                    x += 1/kappa * (-np.sin(yaw) + np.sin(yaw + integration_step * kappa))
                    y += 1/kappa * (np.cos(yaw) - np.cos(yaw + integration_step * kappa))
                    yaw = pify(yaw + integration_step * kappa)
                else:
                    x += integration_step * np.cos(yaw)
                    y += integration_step * np.sin(yaw)

                xs.append(x)
                ys.append(y)
                yaws.append(yaw)

        return xs, ys, yaws

    def plan(self, state1, state2, kappa):
        """
        시작 상태와 목표 상태, 곡률(kappa) 값을 입력받아 Dubins 경로를 생성

        Args:
            state1: 시작 상태 (x, y, yaw)
            state2: 목표 상태 (x, y, yaw)
            kappa: 차량의 곡률 (1/m)

        Returns:
            차량의 이동 경로 (x, y, yaw 좌표) 목록, 제어 입력값 목록, Dubins 경로 객체
        """
        dx = state2[0] - state1[0]
        dy = state2[1] - state1[1]
        th = np.arctan2(dy, dx)

        d = np.hypot(dx, dy) * kappa
        alpha = twopify(state1[2] - th)
        beta = twopify(state2[2] - th)

        dubins_path = self.get_best_dubins_path(d, alpha, beta)
        controls = self.dubins_path_to_controls(dubins_path, kappa)
        cartesian_path = self.controls_to_cartesian_path(controls, state1)

        return cartesian_path, controls, dubins_path

    def plan_waypoint(self, waypoints, kappa):
        """
        여러 개의 지점(waypoint) 목록을 입력받아 각 지점 사이를 연결하는 Dubins 경로를 생성

        Args:
            waypoints: 각 지점의 (x, y, yaw) 좌표를 담은 목록
            kappa: 차량의 곡률 (1/m)

        Returns:
            차량의 이동 경로 (x, y, yaw 좌표) 목록, 제어 입력값 목록, Dubins 경로 객체 목록
        """
        all_xs, all_ys, all_yaws = [], [], []
        all_controls = []
        all_dubins_paths = []

        for i in range(len(waypoints) - 1):
            state1 = waypoints[i]
            state2 = waypoints[i + 1]
            dx = state2[0] - state1[0]
            dy = state2[1] - state1[1]
            th = np.arctan2(dy, dx)

            d = np.hypot(dx, dy) * kappa
            alpha = twopify(state1[2] - th)
            beta = twopify(state2[2] - th)

            dubins_path = self.get_best_dubins_path(d, alpha, beta)
            controls = self.dubins_path_to_controls(dubins_path, kappa)
            cartesian_path = self.controls_to_cartesian_path(controls, state1)

            if cartesian_path:
                xs, ys, yaws = cartesian_path
                all_xs.extend(xs)
                all_ys.extend(ys)
                all_yaws.extend(yaws)
                all_controls.extend(controls)
                all_dubins_paths.append(dubins_path)

        return (all_xs, all_ys, all_yaws), all_controls, all_dubins_paths
