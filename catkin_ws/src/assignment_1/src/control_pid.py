#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class PIDController:
    def __init__(self):
        self.p_gain = 0.15 # default 0.3 0.5 -> 1.0
        self.i_gain = 0.03 # 0.01 -> 0.03
        self.d_gain = 0.03 # default 0.03 0.9 -> 0.7 -> 1.0
        self.prev_error = 0
        self.i_control = 0

    def control(self, target_vel, current_vel, dt):
        error = target_vel - current_vel

        # PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * dt
        d_control = self.d_gain * (error-self.prev_error) / dt

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
