from random import triangular
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
    def draw(self):
        self.screen_projection()

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
        
        rot_mat = self.rotation.transformation_matrix
        
        world_mat = pos_mat @ rot_mat @ scale_mat
        
        return world_mat

    def normalize(self,v):
        return v / np.sqrt(np.sum(v**2)) if np.sqrt(np.sum(v**2)) >= 1e-6 else v

    def screen_projection(self):
        world_mat = self.get_world_mat()
        camera_mat = self.render.camera.camera_matrix()
        projection_mat = self.render.camera.projection_matrix()
        transform_mat = world_mat @ camera_mat @ projection_mat

        #calc normals
        normals = []
        for tri in self.obj.triangles:
            i_0 = tri[0]
            i_1 = tri[1]
            i_2 = tri[2]
            v_0 = self.obj.vertices[i_0]
            v_1 = self.obj.vertices[i_1]
            v_2 = self.obj.vertices[i_2]
            v_a = v_1[:3]-v_0[:3]
            v_b = v_2[:3]-v_0[:3]
            normal = self.normalize(np.cross(v_a, v_b))
            normals.append([*normal,0.0])

        view_normals = list(map(lambda model_n: self.normalize((model_n @ (world_mat@camera_mat))[:3]), normals))
        #moving points important! part
        
        vert = self.obj.vertices @ transform_mat
        vert /= vert[:, -1].reshape(-1, 1)
        #vert[(vert > 2) | (vert < -2)] = 0
        
        vis_tri_idx = []
        n_tri = len(self.obj.triangles)

        for tri_i in range(n_tri):
            triangle = self.obj.triangles[tri_i]
            idx0 = triangle[0]
            idx1 = triangle[1]
            idx2 = triangle[2]

            v0 = vert[idx0]
            v1 = vert[idx1]
            v2 = vert[idx2]


        
        screen_mat =np.array([
            [self.render.H_WIDTH, 0, 0, 0],
            [0, -self.render.H_HEIGHT, 0, 0],
            [0, 0, 1, 0],
            [self.render.H_WIDTH, self.render.H_HEIGHT, 0, 1]
        ])
        vert = vert @ screen_mat
        vert = vert[:,:3]
        for tri in self.obj.triangles:
            poly = vert[tri]
            #normalvec = np.linalg.norm(np.cross( poly[1]-poly[0],poly[2]-poly[0] ))
            #if(np.dot(normalvec,poly[0]-self.position)[0]<0.0):
            poly = poly[:,:2]
            pg.draw.polygon(self.render.screen, pg.Color('yellow'),poly,3)
        print(1)
