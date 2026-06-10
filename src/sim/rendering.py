import pygame 
from sim.constraints import DistanceConstraint
from sim.objects import PointMass

class Renderer():
    def __init__(self, screen, object_list, constraint_list):
        self.screen = screen
        self.object_list = object_list
        self.constraint_list = constraint_list

    def render(self):
        for o in self.object_list:
            if isinstance(o, PointMass):
                pos = self.project_orthographic(o.get_position())
                if o.target_position is not None:
                    target_pos = self.project_orthographic(o.target_position)
                    pygame.draw.circle(self.screen, "red", target_pos, radius = o.radius)
                pygame.draw.circle(self.screen, "black", pos, radius = o.radius)
                
        for c in self.constraint_list:
            if isinstance(c, DistanceConstraint):
                p1 = self.project_orthographic(c.obj1.get_position())
                p2 = self.project_orthographic(c.obj2.get_position())
                
                pygame.draw.line(self.screen, "black", start_pos=p1, end_pos=p2, width=2)

    def project_orthographic(self, point, scale=50):
        x = point[0]
        y = point[1]
        z = point[2]

        x_hat = x * scale + self.screen.get_width() / 2
        y_hat = self.screen.get_height() / 2 - y * scale

        return int(x_hat), int (y_hat)