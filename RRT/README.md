## Implementation of (Informed) RRT*

Scripts:
 - RRT.py (RRT, RRT* and Informed RRT*. Can be used with a holonomic system, simple bicycle model with kinematic constraints, and a more complex model with dynamic constraints)
 - SimplePendulum.py (implemented simple RRT for pendulum specifically. This was the inital version.)
 - DoubleIntegrator2D.py (stopped working on May 16., will not be prioritized to fix)
 
Other:
- All files imports support.py, which implements steering functions for the different systems.
- support.py imports utils and searchClasses, which consists of implementation of plotting tools and classes used for the RRT algorithm
