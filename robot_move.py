import numpy as np
# Move the robot up
def robot_move_theta(robot,l,theta):
    x,y=robot
    robot=(x+l*np.cos(theta),y+l*np.sin(theta))
    cost=1
    return cost,robot
