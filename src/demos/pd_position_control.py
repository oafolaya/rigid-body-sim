from sim.simulation import Simulation
from sim.objects import PointMass
from sim.controllers import PIDController


        
def main():
    pm = PointMass(mass=1.5) 
    pid = PIDController(P=0.5, I=0.0, D=0.35)
    pm.set_target_position([1, 4, 0])
    pid.add_target(pm)
    s = Simulation()
    s.add_object(pm)
    s.add_controller(pid)
    s.toggle_gravity()
    s.run()

  
if __name__ == "__main__":
    main()