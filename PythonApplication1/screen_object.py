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

    def get_z_extents(self, tri):
        tri_array = np.array(tri)  
        z_values = tri_array[:, 2]
        return np.min(z_values), np.max(z_values)

    def check_bounding_box_overlap(self,triangle1, triangle2):
        triangle1 = np.array(triangle1)
        triangle2 = np.array(triangle2)
        min_x1, max_x1 = np.min(triangle1[:, 0]), np.max(triangle1[:, 0])
        min_y1, max_y1 = np.min(triangle1[:, 1]), np.max(triangle1[:, 1])

        min_x2, max_x2 = np.min(triangle2[:, 0]), np.max(triangle2[:, 0])
        min_y2, max_y2 = np.min(triangle2[:, 1]), np.max(triangle2[:, 1])

        if (max_x1 < min_x2 or min_x1 > max_x2) or \
           (max_y1 < min_y2 or min_y1 > max_y2):
            # Bounding boxes do not overlap
            return False
        else:
            # Bounding boxes overlap
            return True

    def screen_projection(self):
        world_mat = self.get_world_mat()
        camera_mat = self.render.camera.camera_matrix()
        projection_mat = self.render.camera.projection_matrix()
        transform_mat = world_mat @ camera_mat

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
            normal = normalize(np.cross(v_a, v_b))
            normals.append([*normal,0.0])

        view_normals = list(map(lambda model_n: normalize((model_n @ (world_mat@camera_mat))[:3]), normals))
        #moving points important! part
        
        vert = self.obj.vertices @ transform_mat
        

        tris = np.copy(self.obj.triangles)
        vis_tri_idx = []
        n_tri = len(tris)
        n_vert = len(vert)
        for tri_i in range(n_tri):

            normal = view_normals[tri_i]
            triangle = tris[tri_i]
            
            idx0 = triangle[0]
            idx1 = triangle[1]
            idx2 = triangle[2]

            v0 = vert[idx0]
            v1 = vert[idx1]
            v2 = vert[idx2]

            if not ((v0[0] * normal[0] + v0[1] * normal[1] + v0[2] * normal[2]) < 0):
                continue

            triss,new_tris,new_vert = clip_tri_plane(np.array([0.0, 0.0, 0.1]), np.array([0.0, 0.0, 1.0]),triangle,np.array([v0[:3],v1[:3],v2[:3]]),n_vert)
            for ver in new_vert:
                vert = np.append(vert,[ver], axis=0)
            for tri in new_tris:
                tris = np.append(tris,[tri], axis=0)
            for tri in triss:
                vis_tri_idx.append(tri)

        z_extents = [self.get_z_extents([vert[tri_n[0]], vert[tri_n[1]], vert[tri_n[2]]]) for tri_n in vis_tri_idx]
        sorted_triangles = []
        # Sort z_extents and triangles based on min_z values
        fortest = True
        while(fortest):
            fortest = False
            if(len(vis_tri_idx)>1):
                sorted_data = sorted(zip(z_extents, vis_tri_idx), key=lambda x: x[0][0])

                # Extract sorted z_extents and triangles
                sorted_z_extents, sorted_triangles = zip(*sorted_data)

                for i in range(len(sorted_z_extents) - 1):
                    curr_extent = sorted_z_extents[i]
                    next_extent = sorted_z_extents[i+1]

                    if curr_extent[1] > next_extent[0]:
                        if self.check_bounding_box_overlap([vert[sorted_triangles[i][0]], vert[sorted_triangles[i][1]], vert[sorted_triangles[i][2]]],[vert[sorted_triangles[i+1][0]], vert[sorted_triangles[i+1][1]], vert[sorted_triangles[i+1][2]]]):
                            triss,new_tris,new_vert = clip_tri_plane(np.array([0.0, 0.0, 0.1]), np.array([0.0, 0.0, 1.0]),triangle,np.array([v0[:3],v1[:3],v2[:3]]),n_vert)
                            #add new clip tri plane for division 
                            for ver in new_vert:
                                vert = np.append(vert,[ver], axis=0)
                            for tri in new_tris:
                                tris = np.append(tris,[tri], axis=0)
                            for tri in triss:
                                vis_tri_idx.append(tri)
                                #if new triangles fortest = true
                
                


        vert = vert @ projection_mat
        vert /= vert[:, -1].reshape(-1, 1)
        screen_mat =np.array([
            [self.render.H_WIDTH, 0, 0, 0],
            [0, -self.render.H_HEIGHT, 0, 0],
            [0, 0, 1, 0],
            [self.render.H_WIDTH, self.render.H_HEIGHT, 0, 1]
        ])
        vert = vert @ screen_mat
        vert = vert[:,:3]
        if(len(sorted_triangles)>0):
            for tri in sorted_triangles:
                poly = vert[tri]
                #normalvec = np.linalg.norm(np.cross( poly[1]-poly[0],poly[2]-poly[0] ))
                #if(np.dot(normalvec,poly[0]-self.position)[0]<0.0):
                poly = poly[:,:2]
                pg.draw.polygon(self.render.screen, pg.Color('yellow'),poly,3)
                pg.draw.polygon(self.render.screen, pg.Color('white'),poly)
            print(1)
