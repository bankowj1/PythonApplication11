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
        self.FPS = 120
        self.screen = pg.display.set_mode(self.RES)
        self.clock = pg.time.Clock()
        self.create_objects()
        self.font = pg.font.SysFont('Arial', 20)

    def create_objects(self):
        self.camera = Camera(self,np.array([0.0,0.0,-4.0]),np.array([1.0,0.0,0.0,1.0]))
        obj = Object3D()
        obj.read_obj_file('ball.obj')
        self.obj1 = ScreenObject(self,obj)


    def draw(self):
        self.screen.fill(pg.Color('darkgray'))
        self.obj1.draw()
        self.print_on_screen('Position')
        self.print_on_screen(str(self.camera.position),offsetY=-20)
        self.print_on_screen('Rotation',offsetY=-40)
        self.print_on_screen(str(self.camera.rotation.yaw_pitch_roll),offsetY=-60)

        
    def print_on_screen(self,strtp,offsetX = 0 , offsetY = 0):
        text_surface = self.font.render(strtp, True, (255, 255, 255))
        text_size = text_surface.get_size()
        x_pos = self.RES[0] - text_size[0] - 10  # 10 is a margin
        y_pos = 10  # also with a margin
        self.screen.blit(text_surface,(x_pos-offsetX,y_pos-offsetY))

    def run(self):
        while True:
            self.draw()
            self.camera.control()
            [exit() for i in pg.event.get() if i.type == pg.QUIT]
            pg.display.set_caption(str(self.clock.get_fps()))
            pg.display.flip()
            self.clock.tick(self.FPS)

if __name__ == '__main__':
    app = Rander()
    app.run()