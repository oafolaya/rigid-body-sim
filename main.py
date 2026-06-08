from abc import ABC, abstractmethod
import numpy as np
import pygame 

pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

# would be cool if i could just make joints a class and do some sort of
# lego type API

class PointMass():
    def __init__(self, mass=10, position = [0, 0, 0], velocity = [0, 0, 0], restitution = 0.6, radius = 5):
        self.mass = mass 
        self.x_position = position[0]
        self.y_position = position[1]
        self.z_position = position[2]
        self.x_velocity = velocity[0]
        self.y_velocity = velocity[1]
        self.z_velocity = velocity[2]
        self.accumulated_force = np.array([0, 0, 0])
        self.target_position = None
        self.restitution = restitution
        self.radius = radius

    def get_state(self): # form state vector
        state = np.array([self.x_position, 
                        self.y_position, 
                        self.z_position,  
                        self.x_velocity, 
                        self.y_velocity, 
                        self.z_velocity])
        state = np.array(state).reshape([len(state), 1])
        return state
    
    def get_position(self):
        position = np.array([self.x_position, self.y_position, self.z_position])
        return position

    def get_velocity(self):
        velocity = np.array([self.x_velocity, self.y_velocity, self.z_velocity])
        return velocity

    def set_state(self, state): # state must be vector of size 6
        state = np.array(state)
        self.x_position = state.item(0)
        self.y_position = state.item(1)
        self.z_position = state.item(2)
        self.x_velocity = state.item(3)
        self.y_velocity = state.item(4)
        self.z_velocity = state.item(5)
    
    def set_position(self, position):
        position = np.array(position)
        self.x_position = position.item(0)
        self.y_position = position.item(1)
        self.z_position = position.item(2)
    
    def set_velocity(self, velocity):
        velocity = np.array(velocity)
        self.x_velocity = velocity.item(0)
        self.y_velocity = velocity.item(1)
        self.z_velocity = velocity.item(2)
        
    def get_mass(self):
        return self.mass
    
    def get_inverse_mass(self):
        return 1/self.mass
    
    def add_force(self, force):
        self.accumulated_force = self.accumulated_force + force
    
    def clear_forces(self):
        self.accumulated_force = np.array([0, 0, 0])

    def set_target_position(self, target_position):
        self.target_position = target_position
    
    def set_restitution(self, restitution):
        self.restitution = restitution

    def set_radius(self, radius):
        self.radius = radius
        
    
class RigidBody():
    def __init__(self, mass=10,moment_of_inertia=10, position = [0, 0, 0],
                velocity = [0, 0, 0], angular_position = [0,0,0],
                angular_velocity = [0, 0, 0], restitution = 0.6):
        self.accumulated_force = np.array([0, 0, 0])
        self.accumulated_torque = np.array([0, 0, 0])
        self.mass = mass
        self.x_position = position[0]
        self.y_position = position[1]
        self.z_position = position[2]
        self.x_velocity = velocity[0]
        self.y_velocity = velocity[1]
        self.z_velocity = velocity[2]
        self.x_angular_position = angular_position[0]
        self.y_angular_position = angular_position[1]
        self.z_angular_position = angular_position[2]
        self.x_angular_velocity = angular_velocity[0]
        self.y_angular_velocity = angular_velocity[1]
        self.z_angular_velocity = angular_velocity[2]
        self.moment_of_inertia = moment_of_inertia
        self.target_position = None
        self.target_angular_position = None
        self.restitution = restitution


    def get_state(self): # form state vector
        state = np.array([self.x_position, 
                        self.y_position, 
                        self.z_position,  
                        self.x_velocity, 
                        self.y_velocity, 
                        self.z_velocity,
                        self.x_angular_position,
                        self.y_angular_position,
                        self.z_angular_position,
                        self.x_angular_velocity,
                        self.y_angular_velocity,
                        self.z_angular_velocity])
        state = np.array(state).reshape([len(state), 1])
        return state

    def set_state(self, state): # state must be vector of size 6
        state = np.array(state)
        self.x_position = state.item(0)
        self.y_position = state.item(1)
        self.z_position = state.item(2)
        self.x_velocity = state.item(3)
        self.y_velocity = state.item(4)
        self.z_velocity = state.item(5)
        self.x_angular_position = state.item(6)
        self.y_angular_position = state.item(7)
        self.z_angular_position = state.item(8)
        self.x_angular_velocity = state.item(9)
        self.y_angular_velocity = state.item(10)
        self.z_angular_velocity = state.item(11)

    def set_position(self, position):
        position = np.array(position)
        self.x_position = position.item(0)
        self.y_position = position.item(1)
        self.z_position = position.item(2)
    
    def set_velocity(self, velocity):
        velocity = np.array(velocity)
        self.x_velocity = velocity.item(0)
        self.y_velocity = velocity.item(1)
        self.z_velocity = velocity.item(2)

    def set_angular_position(self, angular_position):
        position = np.array(angular_position)
        self.x_position = position.item(0)
        self.y_position = position.item(1)
        self.z_position = position.item(2)
    
    def set_angular_velocity(self, angular_velocity):
        velocity = np.array(angular_velocity)
        self.x_velocity = velocity.item(0)
        self.y_velocity = velocity.item(1)
        self.z_velocity = velocity.item(2)
        
    def get_mass(self):
        return self.mass
    
    def get_inverse_mass(self):
        return 1/self.mass
    
    def get_inertia_inverse(self):
        return 1/self.moment_of_inertia
    
    def add_force(self, force):
        self.accumulated_force += force
    
    def clear_force(self):
        self.accumulated_force = np.array([0, 0, 0])

    def add_torque(self, torque):
        self.accumulated_torque += torque
    
    def clear_torque(self,):
        self.accumulated_torque = np.array([0, 0, 0])

    def set_target_position(self, target_position):
        self.target_position = target_position
    
    def set_restitution(self, restitution):
        self.restitution = restitution
    
class Renderer():
    def __init__(self, screen, object_list):
        self.screen = screen
        self.object_list = object_list

    def render(self):
        for o in self.object_list:
            if isinstance(o, PointMass):
                pos = self.project_orthographic(o.get_position())
                if o.target_position is not None:
                    target_pos = self.project_orthographic(o.target_position)
                pygame.draw.circle(self.screen, "black", pos, radius = o.radius)
                pygame.draw.circle(self.screen, "red", target_pos, radius = o.radius)

    def project_orthographic(self, point, scale=50):
        x = point[0]
        y = point[1]
        z = point[2]

        x_hat = x * scale + screen.get_width() / 2
        y_hat = screen.get_height() / 2 - y * scale

        return int(x_hat), int (y_hat)




class Controller(ABC):
    @abstractmethod
    def compute_control(self, state, reference, t, dt):
        pass

class PIDController(Controller):
    def __init__(self, P=1.0, I=0.0, D=0.0):
        self.P = P
        self.I = I
        self.D = D
        self.integral = 0
        self.targets = None
        self.last_error = 0

    def compute_control(self, state, reference, t, dt):
        error = reference - state
        derivative = (error - self.last_error) / dt 
        self.integral += error * dt
        control = self.P * error + self.I * self.integral + self.D * derivative
        self.last_error = error
        return control
    
    def add_target(self, object):
        if self.targets == None:
            self.targets = [object]
        else:
            self.targets.append(object)


class Integrator(ABC):
    @abstractmethod
    def step(self, x, xdot, dt):
        pass

class EulerIntegrator(Integrator):
    def step(self, x, xdot, dt):
        x = x + xdot * dt
        return x

        
class Simulation():
    def __init__(self, width=1280, height=720, integrator = EulerIntegrator(), scale=50, gravity_on = True):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.object_list = []
        self.renderer = Renderer(self.screen, self.object_list)
        self.gravity_on = gravity_on
        self.gravity = np.array([0, -9.81, 0])
        self.integrator = integrator
        self.controller_list = []
        self.t = 0
        self.scale = scale
        self.x_boundary = [-screen.get_width()/(2*self.scale), screen.get_width()/(2 * self.scale)]
        self.y_boundary = [-screen.get_height()/(2*self.scale), screen.get_height()/(2 * self.scale)]
    
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
    
    def handle_collisions(self):
        for o in self.object_list:
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

    def run(self):
        done = False
        self.t = 0
        while not done:
            dt = clock.tick(60) / 1000
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
            screen.fill("white")
            
            

            self.clear_forces()
            self.apply_world_forces()
            self.compute_control(dt)
            self.integrate(dt)
            self.handle_collisions()
            self.renderer.render()

            self.t += dt
            print(self.t)
            pygame.display.flip()

        pygame.quit()
    
    def add_object(self, object):
        self.object_list.append(object)
    
    def add_controller(self, controller):
        self.controller_list.append(controller)

        

class Link():
    pass

class Finger():
    def __init__(self, n_joints):
        self.num_joints = n_joints
        self.limits = [(0, 180), (0, 90), (0, 90)] # degrees
        self.lengths = [4, 3, 2] # m
        self.theta = [0, 0, 0]


def main():
    pm = PointMass(mass=10, position=[0, 0, 0])
    pm.set_target_position([2, 2, 1])
    pid = PIDController(P = 0.8, I = 0.0, D=0.4)
    pid.add_target(pm)

    s = Simulation()
    s.add_object(pm)
    s.add_controller(pid)
    s.toggle_gravity()
    s.run()

  
if __name__ == "__main__":
    main()