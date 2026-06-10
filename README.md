# Simple Controls/Physics Simulation Framework in Pygame and Numpy
Simple Controls/Physics Simulation Framework in Python, Pygame, and Numpy.

The goal of this project is to build a modular physics simulation tool for experimenting with dynamics, contraints, and controllers. I am implementing the core simulation components from scratch to better understand constraint solving and articulated dynamics. Ideally, it becomes a platform where I can test modern control techniques on rigid-bodies while learning the ins and outs of physical modeling, constraint handling, dynamics.
git .
## Progress
So far, I have implemented the following:
- Point Mass Object
- Basic 2D Rendering using Pygame
- PID Control
- Jacobian-based constraint framework
- Fixed Point Constraint
- Distance Constraint
- Baumgarte stabilization 
- Real-time visualization using Pygame

My Immediate Next Steps:
- Implement Wall/Boundary Constraints (in the same generic form as the Fixed Distance and Fixed Point Constraints)
- Implement Fixed and Revolute Joint Class w/ joint limits
- Complete Rigid Body Support
- Build articulated mechanisms (robotic fingers and linkage systems)
- Add Camera abstractions and improve visualization
- Add logging and metrics for controller evaluation


