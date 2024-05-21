#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from math import *

class VehicleModel:
    def __init__(self, length=50, width=30, axle_ratio=1/2, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.length = length
        self.width = width
        self.axle_ratio = axle_ratio
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def get_front_coordinates(self):
        # Calculate the center coordinates from the rear axle coordinates
        front_x = self.x + (self.length * self.axle_ratio) * cos(self.yaw)
        front_y = self.y + (self.length * self.axle_ratio) * sin(self.yaw)
        return front_x, front_y

    def get_rear_coordinates(self):
        # Calculate the center coordinates from the rear axle coordinates
        rear_x = self.x - (self.length * self.axle_ratio) * cos(self.yaw)
        rear_y = self.y - (self.length * self.axle_ratio) * sin(self.yaw)
        return rear_x, rear_y

    def plot_vehicle(self, ax):
        x, y, yaw = self.x, self.y, self.yaw
        length, width = self.length, self.width
        front_x, front_y = self.get_front_coordinates()
        rear_x, rear_y = self.get_rear_coordinates()

        # Calculate corner coordinates
        corners = np.array([
            [length/2, width/2],
            [length/2, -width/2],
            [-length/2, -width/2],
            [-length/2, width/2]
        ])

        # Rotation matrix
        R = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)]
        ])

        # Rotate and translate corners
        transformed_corners = np.dot(corners, R) + np.array([x, y])

        # Close the rectangle by appending the first corner to the end
        transformed_corners = np.vstack([transformed_corners, transformed_corners[0]])

        # Plot vehicle rectangle
        ax.plot(front_x, front_y, 'bo', label='car_front')
        ax.plot(rear_x, rear_y, 'bo', label='car_rear')
        ax.plot(transformed_corners[:, 0], transformed_corners[:, 1], 'k-')
        ax.fill(transformed_corners[:, 0], transformed_corners[:, 1], 'gray', alpha=0.5)
