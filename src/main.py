from sim.simulation import Simulation
from sim.objects import PointMass
from sim.constraints import FixedPointConstraint, DistanceConstraint
from sim.controllers import PIDController

        
def main():
    pm = PointMass(mass=1.5) 
    m = PointMass(mass=10.0, position=[1,5,0])
    d = DistanceConstraint(pm, m, 3)
    pid = PIDController(P=1, I = 1)
    fix = FixedPointConstraint(pm, [0, 0, 0])
    pid.add_target(pm)
    pm.set_target_position([5,5,0])

    s = Simulation()
    s.add_constraint(d)
    s.add_object(pm)
    s.add_object(m)
    s.add_controller(pid)
    s.run()

  
if __name__ == "__main__":
    main()