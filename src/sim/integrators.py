from abc import ABC, abstractmethod

class Integrator(ABC):
    @abstractmethod
    def step(self, x, xdot, dt):
        pass

class EulerIntegrator(Integrator):
    def step(self, x, xdot, dt):
        x = x + xdot * dt
        return x

