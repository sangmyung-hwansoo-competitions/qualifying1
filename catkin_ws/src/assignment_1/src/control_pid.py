#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class PIDController:
    """
    PID 제어기를 구현하는 클래스 

    PID 제어기 == 비례적분미분 제어기(Proportional Integral Derivative Controller)
    시스템의 출력을 원하는 목표 값 (target)에 근사화함

    """
    def __init__(self):
        """
        생성자 함수
        PID controller에 사용되는 gain 값들을 초기화

        Args:
            없음
        """
        self.p_gain = 1.8       # default 0.3 0.5 -> 1.0 (비례 gain)
        self.i_gain = 0.3       # 0.01 -> 0.03 (적분 gain)
        self.d_gain = 1.0       # default 0.03 0.9 -> 0.7 -> 1.0 (미분 gain)
        self.prev_error = 0     # 이전 오차 값
        self.i_control = 0      # 적분 제어 값

    def control(self, target_vel, current_vel, dt):
        """
        PID control 값 계산하는 함수

        Args:
            target_vel (float): 목표 속도
            current_vel (float): 현재 속도
            dt (float): 시간 간격

        Returns:
            float: 계산된 제어 값
        """
        # 오차 계산
        error = target_vel - current_vel

        # PID 제어 생성
        p_control = self.p_gain * error                         # 비례 제어
        self.i_control += self.i_gain * error * dt              # 적분 제어 (누적 오차)
        d_control = self.d_gain * (error-self.prev_error) / dt  # 미분 제어 (오차 변화율)

        output = p_control + self.i_control + d_control         # PID control 값 sum
        self.prev_error = error                                 # 다음 계산을 위해 현재 오차값 저장

        return output
