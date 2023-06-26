from pyquaternion import Quaternion
from matrix_math import *
from numpy import ndarray
import math
import numpy as np
import pygame as pg

class Camera:
    def __init__(self, render, position: ndarray = np.array([0.0, 0.0, 0.0]), 
                 rotation: ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
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
        self.moving_speed = 0.1
        self.rotation_speed = 0.015
    def move(self,v):
        q = Quaternion(0.0,*v)

        res = self.rotation.conjugate * q * self.rotation

        return np.array([res.x,res.y,-res.z])

    def control(self):
        key = pg.key.get_pressed()
        mods = pg.key.get_mods()
        if key[pg.K_a]:
            if mods & pg.KMOD_SHIFT:
                rotation = Quaternion._from_axis_angle(np.array([0.0, math.radians(90), 0.0]),self.rotation_speed)
                self.camera_rotate(rotation)
            else:
                v = self.move(np.array([self.moving_speed, 0.0,0.0]))
                self.position += v
        if key[pg.K_d]:
            if mods & pg.KMOD_SHIFT:
                rotation = Quaternion._from_axis_angle(np.array([0.0, math.radians(90), 0.0]),-self.rotation_speed)
                self.camera_rotate(rotation)
            else:
                v = self.move(np.array([-self.moving_speed, 0.0,0.0]))
                self.position += v
        if key[pg.K_w]:
            if mods & pg.KMOD_SHIFT:
                rotation = Quaternion._from_axis_angle(np.array([math.radians(90), 0.0, 0.0]),self.rotation_speed)
                self.camera_rotate(rotation)
            else:
                v = self.move(np.array([ 0.0,0.0,self.moving_speed]))
                self.position += v
        if key[pg.K_s]:
            if mods & pg.KMOD_SHIFT:
                rotation = Quaternion._from_axis_angle(np.array([math.radians(90), 0.0, 0.0]),-self.rotation_speed)
                self.camera_rotate(rotation)
            else:
                v = self.move(np.array([ 0.0,0.0,-self.moving_speed]))
                self.position += v
        if key[pg.K_q]:
            if mods & pg.KMOD_SHIFT:
                rotation = Quaternion._from_axis_angle(np.array([0.0, 0.0, math.radians(90)]),self.rotation_speed)
                self.camera_rotate(rotation)
            else:
                v = self.move(np.array([ 0.0,self.moving_speed,0.0]))
                self.position += v
        if key[pg.K_e]:
            if mods & pg.KMOD_SHIFT:
                rotation = Quaternion._from_axis_angle(np.array([0.0, 0.0,math.radians(90)]),-self.rotation_speed)
                self.camera_rotate(rotation)
            else:
                v = self.move(np.array([ 0.0,-self.moving_speed,0.0]))
                self.position += v

    def camera_rotate(self, angle):
        print(angle)
        print(self.rotation)
        print(angle.normalised)
        print(self.rotation.normalised)
        print((self.rotation*angle.normalised).normalised)
        self.rotation = (self.rotation*angle.normalised).normalised


    def axiiIdentity(self):
        self.forward = np.array([1,0, 0, 1])
        self.up = np.array([1,0, 1, 0])
        self.right = np.array([1,1, 0, 0])

    def camera_update_axii(self):
        rotate = self.rotation.transformation_matrix
        self.axiiIdentity()
        self.forward = self.forward @ rotate
        self.right = self.right @ rotate
        self.up = self.up @ rotate

    def camera_matrix(self):
        self.camera_update_axii()
        return np.linalg.inv(self.rotation.transformation_matrix@translate(self.position))

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








