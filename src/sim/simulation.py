import pygame 
import numpy as np
from sim.rendering import Renderer
from sim.integrators import EulerIntegrator
from sim.objects import PointMass, RigidBody
from sim.constraints import StaticPlane


class Simulation():
    def __init__(self, width=1280, height=720, integrator = EulerIntegrator(), scale=50, gravity_on = True):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.object_list = []
        self.constraint_list = []
        self.static_plane_list = []
        self.renderer = Renderer(self.screen, self.object_list, self.constraint_list, scale)
        self.gravity_on = gravity_on
        self.gravity = np.array([0, -9.81, 0])
        self.integrator = integrator
        self.controller_list = []
        self.t = 0
        self.dt = None
        self.scale = scale
        self.x_boundary = [-self.screen.get_width()/(2*self.scale), self.screen.get_width()/(2 * self.scale)]
        self.y_boundary = [-self.screen.get_height()/(2*self.scale), self.screen.get_height()/(2 * self.scale)]
    
    def world_to_screen_position(self, position):
        x = position.item(0)
        y = position.item(1)
        z = position.item(2) # Z should influence x_screen and y_screen (will do once camera object implemented)

        x_screen = x*self.scale + (self.screen.get_width()/2) 
        y_screen = y*self.scale + (self.screen.get_height()/2) 

        return x_screen, y_screen

    def set_scale(self, scale):
        # Scale converts world units to pixels
        self.scale = scale 
    
    def toggle_gravity(self):
        if self.gravity_on == True:
            self.gravity_on = False
        else:
            self.gravity_on = True

    def compute_control(self, dt):
        for c in self.controller_list:
            for o in c.targets:
                position = o.get_position()
                position_target = o.target_position
                force = c.compute_control(position, position_target, self.t, dt)
                o.add_force(force)

                if isinstance(o, PointMass) == False:
                    if o.target_angular_position == None:
                        continue
                    angular_position = o.angular_position
                    angular_position_target = o.angular_position_target
                    torque = c.compute_control(angular_position, angular_position_target, self.t, dt)
                    o.add_torque(torque) 

    def clear_forces(self):
        for o in self.object_list:
            o.clear_forces()
            if isinstance(o, RigidBody):
                o.clear_torque()
    
    def apply_world_forces(self):
        for o in self.object_list:
            if self.gravity_on == True:
                gravitational_force = self.gravity * o.mass
                o.add_force(gravitational_force)
        
        # need to add collision system or something
    
    def integrate(self, dt):
        for o in self.object_list:
            position = o.get_position()
            velocity = o.get_velocity()
            acceleration = o.get_inverse_mass() * o.accumulated_force
            new_velocity = self.integrator.step(velocity, acceleration, dt)
            new_position = self.integrator.step(position, velocity, dt)
            o.set_velocity(new_velocity)
            o.set_position(new_position)

            if isinstance(o, PointMass) == False:
                angular_position = o.angular_position
                angular_velocity = o.angular_velocity
                torque = o.accumulated_torque
                angular_acceleration = o.get_inertia_inverse() * torque
                new_angular_velocity = self.integrator.step(angular_velocity, angular_acceleration, dt)
                new_angular_position = self.integrator.step(angular_position, angular_velocity, dt)
                o.set_angular_velocity(new_angular_velocity)
                o.set_angular_position(new_angular_position)
    
    def handle_collisions(self, dt):
        for o in self.object_list:
            for c in self.static_plane_list:
                c.set_object(o)
                c.solve(dt = self.dt)
            """
            position = o.get_position()
            if isinstance(o, PointMass):
                radius_world = o.radius / self.scale
                if (position.item(0) - radius_world) <= self.x_boundary[0]:
                    o.x_velocity = o.x_velocity * -o.restitution
                    o.x_position = self.x_boundary[0] + radius_world
                
                if (position.item(0) + radius_world) >= self.x_boundary[1]:
                    o.x_velocity = o.x_velocity * -o.restitution
                    o.x_position = self.x_boundary[1] - radius_world
                
                if (position.item(1) - radius_world) <= self.y_boundary[0]:
                    o.y_velocity = o.y_velocity * -o.restitution
                    o.y_position = self.y_boundary[0] + radius_world
                
                if (position.item(1) + radius_world) >= self.y_boundary[1]:
                    o.y_velocity = o.y_velocity * -o.restitution
                    o.y_position = self.y_boundary[1] - radius_world
            """
    def handle_constraints(self, iterations=5):
        for c in self.constraint_list:
            for i in range(iterations):
                c.solve(dt=self.dt)
    
    def initialize_static_planes(self):
        left   = StaticPlane([1, 0, 0],  [self.x_boundary[0], 0, 0], scale=self.scale)
        right  = StaticPlane([-1, 0, 0], [self.x_boundary[1], 0, 0], scale=self.scale)
        bottom = StaticPlane([0, 1, 0],  [0, self.y_boundary[0], 0], scale=self.scale)
        top    = StaticPlane([0, -1, 0], [0, self.y_boundary[1], 0], scale=self.scale)
        self.static_plane_list.append(left)
        self.static_plane_list.append(right)
        self.static_plane_list.append(bottom)
        self.static_plane_list.append(top)
        

    def run(self):
        done = False
        self.t = 0

        self.initialize_static_planes()
        self.initialize_constraints()

        while not done:
            self.dt = self.clock.tick(60) / 1000
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
            self.screen.fill("white")
            
            
            self.apply_world_forces()
            self.compute_control(self.dt)
            self.integrate(self.dt)
            self.clear_forces()
            self.handle_collisions(dt= self.dt)
            self.handle_constraints()
            self.renderer.render()

            self.t += self.dt
            #print(self.t)
            pygame.display.flip()

        pygame.quit()
    
    def initialize_constraints(self):
        for c in self.constraint_list:
            c.initialize_constraint()

    def add_object(self, object):
        self.object_list.append(object)
    
    def add_controller(self, controller):
        self.controller_list.append(controller)
    
    def add_constraint(self, constraint):
        self.constraint_list.append(constraint)