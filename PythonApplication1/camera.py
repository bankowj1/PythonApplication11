from pyquaternion import Quaternion
from matrix_math import *
from numpy import ndarray
import math
import numpy as np
import pygame as pg

class Camera:
    def __init__(self, render, position: ndarray = np.array([0, 0, 0]), 
                 rotation: ndarray = np.array([1, 0, 0, 0])):
        self.render = render
        self.position = position
        self.rotation = Quaternion(rotation).normalised
        self.forward = np.array([0, 0, 1, 1])
        self.up = np.array([0, 1, 0, 1])
        self.right = np.array([1, 0, 0, 1])
        self.fov = 90
        self.aspect = render.HEIGHT / render.WIDTH
        self.near_plane = 0.1
        self.far_plane = 100
        self.moving_speed = 0.3
        self.rotation_speed = 0.015

    def control(self):
        key = pg.key.get_pressed()
        if key[pg.K_a]:
            self.position -= self.right * self.moving_speed
        if key[pg.K_d]:
            self.position += self.right * self.moving_speed
        if key[pg.K_w]:
            self.position += self.forward * self.moving_speed
        if key[pg.K_s]:
            self.position -= self.forward * self.moving_speed
        if key[pg.K_q]:
            self.position += self.up * self.moving_speed
        if key[pg.K_e]:
            self.position -= self.up * self.moving_speed

        if key[pg.K_LEFT]:
            self.camera_yaw(-self.rotation_speed)
        if key[pg.K_RIGHT]:
            self.camera_yaw(self.rotation_speed)
        if key[pg.K_UP]:
            self.camera_pitch(-self.rotation_speed)
        if key[pg.K_DOWN]:
            self.camera_pitch(self.rotation_speed)

    def camera_rotate(self, angle):
        self.rotation = (self.rotation*angle).normalised


    def axiiIdentity(self):
        self.forward = np.array([0, 0, 1])
        self.up = np.array([0, 1, 0])
        self.right = np.array([1, 0, 0])

    def camera_update_axii(self):
        rotate = self.rotation.rotation_matrix
        self.axiiIdentity()
        self.forward = self.forward @ rotate
        self.right = self.right @ rotate
        self.up = self.up @ rotate

    def camera_matrix(self):
        self.camera_update_axii()
        return translate(self.position) @ self.rotation.rotation_matrix

    def projection_matrix(self):
        m00 = self.aspect*(1/math.tan(math.radians(self.fov/ 2)))
        m11 = (1/math.tan(math.radians(self.fov/ 2)))
        m22 = (self.far_plane + self.near_plane) / (self.far_plane - self.near_plane)
        m32 = -2 * self.far_plane * self.near_plane / (self.far_plane - self.near_plane)
        return np.array([
            [m00, 0, 0, 0],
            [0, m11, 0, 0],
            [0, 0, m22, 1],
            [0, 0, m32, 0]
        ])








