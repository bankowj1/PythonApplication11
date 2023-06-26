from matrix_math import *
import numpy as np


class Object3D:
    def __init__(self, vertices: np.ndarray, triangles: np.ndarray):
        self.vertices = vertices
        self.triangles = triangles
    def __init__(self):
        pass
    def get_vertices(self):
        return self.vertices

    def set_vertices(self, vertices):
        self.vertices = vertices

    def get_triangles(self):
        return self.triangles

    def set_triangles(self, triangles):
        self.triangles = triangles

    def read_obj_file(self, filename):
        vertices = []
        triangles = []

        with open(filename) as f:
            for line in f:
                if line.startswith('v '):
                    vertices.append([float(i) for i in line.split()[1:]] + [1])
                elif line.startswith('f'):
                    faces_ = line.split()[1:]
                    triangles.append([int(face_.split('/')[0]) - 1 for face_ in faces_])

        self.vertices = np.array(vertices)
        self.triangles = np.array(triangles)

    def write_obj_file(self, filename):
        with open(filename, 'w') as f:
            for vertex in self.vertices:
                f.write('v {} {} {}\n'.format(vertex[0], vertex[1], vertex[2]))
            for triangle in self.triangles:
                f.write('f {} {} {}\n'.format(triangle[0], triangle[1], triangle[2]))




