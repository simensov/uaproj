## Implementation of (Informed) RRT*

Scripts:
 - RRT.py (RRT, RRT* and Informed RRT*. Can be used with a holonomic system, simple bicycle model with kinematic constraints, and a more complex model with dynamic constraints)
 - SimplePendulum.py (implemented simple RRT for pendulum specifically. This was the inital version.)
 - DoubleIntegrator2D.py (stopped working on May 16., will not be prioritized to fix)
 
Other:
- All files imports support.py, which implements steering functions for the different systems.
- support.py imports utils and searchClasses, which consists of implementation of plotting tools and classes used for the RRT algorithm
- An example of differential flatness is added, but will not run in this folder due to a lot of reasons, first and foremost since the support files made for it is in another directory, but also since the user don't have PYTHONPATH set correctly. Added due to display of an example discussed in the final report.
