from screen_object import *
from object_3d import *
from camera import *
import pygame as pg
import numpy as np


class Rander:
    def __init__(self):
        pg.init()
        self.RES = self.WIDTH, self.HEIGHT = 1600,900
        self.H_WIDTH, self.H_HEIGHT = self.WIDTH // 2, self.HEIGHT // 2
        self.FPS = 165
        self.screen = pg.display.set_mode(self.RES)
        self.clock = pg.time.Clock()
        self.create_objects()

    def create_objects(self):
        self.camera = Camera(self,np.array([0,0,-4]),np.array([1,0,0,1]))
        obj = Object3D()
        obj.read_obj_file('/cube.obj')
        self.obj1 = ScreenObject()


    def draw(self):
        self.screen.fill(pg.Color('darkgray'))

    def run(self):
        while True:
            self.draw()
            [exit() for i in pg.event.get() if i.type == pg.QUIT]
            pg.display.set_caption(str(self.clock.get_fps()))
            pg.display.flip()
            self.clock.tick(self.FPS)

if __name__ == '__main__':
    app = Rander()
    app.run()