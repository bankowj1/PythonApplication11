from object_3d import Object3D
from numpy import ndarray
from pyquaternion import Quaternion
from matrix_math import *
import numpy as np
import pygame as pg

class ScreenObject:
    def __init__(self,render, obj: Object3D, position: ndarray = np.array([0, 0, 0]), 
                 scaleObj: ndarray = np.array([1, 1, 1]), rotation: ndarray = np.array([1, 0, 0, 0])):
        self.render = render
        self.obj = obj
        self.position = position
        self.scaleObj = scaleObj
        self.rotation = Quaternion(rotation)

    def get_object(self):
        return self.obj

    def get_position(self):
        return self.position

    def set_position(self, position):
        self.position = position

    def get_scale(self):
        return self.scale

    def set_scale(self, scale):
        self.scale = scale

    def get_rotation(self):
        return self.rotation

    def set_rotation(self, rotation):
        self.rotation = rotation

    def translate(self):
        self.obj.vertices = self.obj.vertices @ translate(self.position)

    def scale(self):
        self.obj.vertices = self.obj.vertices @ scale(self.scaleObj)

    def rotate(self):
        self.obj.vertices = self.rotation.rotate(self.obj.vertices)

    def get_world_mat(self):    
        # Convert the position, scale, and rotation to matrices
        pos_mat = translate(self.position)
        
        scale_mat = translate(self.position)
        
        rot_mat = self.rotation.rotation_matrix
        
        world_mat = pos_mat @ rot_mat @ scale_mat
        
        return world_mat

    def screen_projection(self):
        world_mat = self.get_world_mat()
        camera_mat = self.render.camera.camera_matrix()
        projection_mat = self.render.camera.projection_matrix()
        transform_mat = world_mat @ camera_mat @ projection_mat
        vert = self.obj.vertices @ transform_mat
        for tri in self.obj.triangles:
            poly = vert[tri]
            pg.draw.polygon(self.render.screen, pg.Color('yellow'),poly,3)
        print(1)
