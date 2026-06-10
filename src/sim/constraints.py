import numpy as np
from abc import ABC, abstractmethod

class Constraint(ABC):
    @abstractmethod
    def compute_jacobian(self):
        pass

    @abstractmethod
    def solve(self, baumgarte, dt):
        pass

class FixedPointConstraint(Constraint):
    def __init__(self, obj, fixed_point):
        self.obj = obj
        self.fixed_point = fixed_point
    
    def compute_jacobian(self):
        J = np.identity(3)
        return J
    
    def solve(self, baumgarte=0.1, dt =1.0):
        J = self.compute_jacobian()
        p = self.obj.get_position()
        v = self.obj.get_velocity()
        C = p - self.fixed_point
        m = self.obj.mass   
        M = np.array([[m, 0, 0],
                    [0, m, 0],
                    [0, 0, m]])
        A = J @ np.linalg.inv(M) @ J.T
        B = J @ v + baumgarte*C/dt
        lmda = -np.linalg.inv(A) @ B
        impulse = J.T @ lmda
        new_v = v + np.linalg.inv(M) @ impulse
        self.obj.set_velocity(new_v)
    
    def initialize_constraint(self):
        self.obj.set_position(np.array(self.fixed_point))


class DistanceConstraint(Constraint):
    def __init__(self, obj1, obj2, distance):
        self.obj1 = obj1
        self.obj2 = obj2
        self.distance = distance
        
    def compute_jacobian(self):
        p1 = self.obj1.get_position()
        p2 = self.obj2.get_position()
        n = (p2 - p1) / np.linalg.norm(p2-p1)
        J = np.array([-n.item(0),-n.item(1), -n.item(2), n.item(0), n.item(1), n.item(2)]).ravel()
        return J
    
    def solve(self, baumgarte=0.2, dt=1.0):
        p1 = self.obj1.get_position()
        p2 = self.obj2.get_position()
        C = np.linalg.norm(p2-p1) - self.distance
        J = self.compute_jacobian()
        v1 = self.obj1.get_velocity()
        v2 = self.obj2.get_velocity()
        V = np.array([v1.item(0), v1.item(1), v1.item(2), v2.item(0), v2.item(1), v2.item(2)])
        m1 = float(self.obj1.mass)
        m2 = float(self.obj2.mass)
        M =  np.array([[m1, 0, 0, 0, 0, 0],
                      [0, m1, 0, 0, 0, 0],
                      [0, 0, m1, 0, 0, 0],
                      [0, 0, 0, m2, 0, 0],
                      [0, 0, 0, 0, m2, 0],
                      [0, 0, 0, 0, 0, m2]])
        
        Minv =  np.array([[1/m1, 0, 0, 0, 0, 0],
                      [0, 1/m1, 0, 0, 0, 0],
                      [0, 0, 1/m1, 0, 0, 0],
                      [0, 0, 0, 1/m2, 0, 0],
                      [0, 0, 0, 0, 1/m2, 0],
                      [0, 0, 0, 0, 0, 1/m2]])
        
        lmda = -(1/((J @ np.linalg.inv(M)) @ J.T)) * ((J.T @ V) + baumgarte*C/dt)
        impulse = J.T * lmda
        V = V + Minv @ impulse
        newv1 = V[0:3]
        newv2 = V[3:6]
        self.obj1.set_velocity(newv1)
        self.obj2.set_velocity(newv2)
    
    def initialize_constraint(self):
        p1 = self.obj1.get_position()
        p2 = self.obj2.get_position()
        init_distance = np.linalg.norm(p2-p1)
        if init_distance != 0:
            normal_dir = (p2 - p1) / init_distance
            p2 = p1 + normal_dir*self.distance
            self.obj2.set_position(p2)
        else:
            new_p2 = np.array([p1.item(0)+5, p1.item(1), p1.item(2)])
            self.obj2.set_position(new_p2)