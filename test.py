from KinematicBicycle import *


environment = Environment('env_superbug.yaml')
start = (0, 0)
radius = 0.3
bounds = (-2, -3, 12, 8)
goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])

bike = KinematicBicycle()

path = bike.rrt(bounds, environment,start, radius, goal_region,start_theta=np.pi,plot=True)

plt.show()