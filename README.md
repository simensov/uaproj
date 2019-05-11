# 6.832 Underactuated Robotics project, Spring 2019

Investigated RRT* for motion planning with differential and dynamic constraints, inspired by "Kinodynamic RRT* Optimal Motion Planning for Systems with Linear Differential Constraints" (Dustin J. Webb, Jur van den Berg, 2012), https://arxiv.org/abs/1205.5088v1.

Another part in the project was done by implementing ideas from "Real-time motion planning for agile autonomous vehicles" (Emilio Frazzoli et. al., 2002), https://ieeexplore.ieee.org/document/945511.

## File structure:

Run scripts:
 - RRT.py (can be used with bicycle with kinematic and dynamic constraints)
 - DoubleIntegrator2D.py
 - SimplePendulum.py

Other:
- All files imports support.py, which implements steeringfunctions
- support.py imports utils and searchClasses, which consists of implementation of plotting tools and classes used for the RRT search
