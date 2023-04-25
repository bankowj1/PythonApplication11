from matrix_math import *
import numpy as np


class Object3D:
    def __init__(self, vertices: np.ndarray, triangles: np.ndarray):
        self.vertices = vertices
        self.triangles = triangles
    def __init__(self):

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

        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('v '):
                    vertex = tuple(map(float, line[2:].split()))
                    vertices.append(vertex)
                elif line.startswith('f '):
                    triangle = tuple(map(int, line[2:].split()))
                    triangles.append(triangle)

        self.vertices = np.array(vertices)
        self.triangles = np.array(triangles)

    def write_obj_file(self, filename):
        with open(filename, 'w') as f:
            for vertex in self.vertices:
                f.write('v {} {} {}\n'.format(vertex[0], vertex[1], vertex[2]))
            for triangle in self.triangles:
                f.write('f {} {} {}\n'.format(triangle[0], triangle[1], triangle[2]))




