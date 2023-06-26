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
            for tri in triss:
                vis_tri_idx.append(tri)


        z_extents = [self.get_z_extents([vert[tri_n[0]], vert[tri_n[1]], vert[tri_n[2]]]) for tri_n in vis_tri_idx]
        
        sorted_triangles = []

        if(len(vis_tri_idx)>1):
            sorted_data = sorted(zip(z_extents, vis_tri_idx), key=lambda x: x[0][0])

            # Extract sorted z_extents and triangles
            sorted_z_extents, sorted_triangles = zip(*sorted_data)
            sorted_triangles = list(map(list, sorted_triangles))
            sorted_z_extents = list(map(list, sorted_z_extents))
            oldsort = sorted_triangles.copy()
            oldz = sorted_z_extents.copy()
            sorted_triangles = [oldsort[0]]
            sorted_z_extents = [oldz[0]]
            for i in range(len(oldz) - 1):
                next_extent = oldz[i+1]
                next_v0 = vert[oldsort[i+1][0]]
                next_v1 = vert[oldsort[i+1][1]]
                next_v2 = vert[oldsort[i+1][2]]
                g = 0
                add = True
                for j in range(0,len(sorted_z_extents) - 1):
                    curr_extent = sorted_z_extents[j]
                    curr_v0 = vert[sorted_triangles[j][0]]
                    curr_v1= vert[sorted_triangles[j][1]]
                    curr_v2= vert[sorted_triangles[j][2]]
                    
                    normal = normalize(np.cross(curr_v1[:3]-curr_v0[:3], curr_v2[:3]-curr_v0[:3]))
                    front = devide_tri_plane(curr_v0[:3],normal,oldsort[i+1],np.array([next_v0[:3],next_v1[:3],next_v2[:3]]),n_vert)
                    if (front==0):
                        sorted_triangles.insert(j,oldsort[i+1])
                        sorted_z_extents.insert(j,oldz[i+1])
                        add = False
                        break
                    if(front == 3):
                        continue
                    normal = normalize(np.cross(next_v1[:3]-next_v0[:3], next_v2[:3]-next_v0[:3]))
                    front = devide_tri_plane(next_v0[:3],normal,oldsort[i+1],np.array([curr_v0[:3],curr_v1[:3],curr_v2[:3]]),n_vert)
                    if(front == 3):
                        sorted_triangles.insert(j,oldsort[i+1])
                        sorted_z_extents.insert(j,oldz[i+1])
                        add = False
                        break


                if(add):
                    sorted_triangles.append(oldsort[i+1])
                    sorted_z_extents.append(oldz[i+1])
                            



                                
                            

                    
                    


                            
                            
                
                


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
        i = 0
        pos_nap = []
        if(len(sorted_triangles)>0):
            for tri in sorted_triangles:
                poly = vert[tri]
                #normalvec = np.linalg.norm(np.cross( poly[1]-poly[0],poly[2]-poly[0] ))
                #if(np.dot(normalvec,poly[0]-self.position)[0]<0.0):
                poly = poly[:,:2]
                pg.draw.polygon(self.render.screen, pg.Color('red'),poly,3)
                pg.draw.polygon(self.render.screen, pg.Color('white'),poly)
                center_x = sum(point[0] for point in poly) // 3
                center_y = sum(point[1] for point in poly) // 3
                pos_nap.append([center_x,center_y])
                
                
                # Render the number and display it in the center
            for nap in pos_nap:
                text = self.render.font.render(str(i), True, pg.Color('black'))
                text_rect = text.get_rect(center=(nap[0], nap[1]))
                #self.render.screen.blit(text, text_rect)
                i += 1
            print(1)
