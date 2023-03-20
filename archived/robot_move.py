import numpy as np
import math
# Move the robot up
stepsize = 10
xg = 90
yg = 90
def move_robot(x,y,curr_theta, totalcst):
    new_nodes = []
    for t in range(-60,61,30):
        x_t,y_t, t_t, c2g, c2c=  actions(x,y,t,curr_theta, totalcst)
        new_nodes.append([c2g+c2c,c2c,c2g,(x_t,y_t,t_t)])
    return new_nodes

def euclidean(x,y,xg,yg):
    return math.dist((x,y),(xg,yg))

def actions(x,y,t,ct, tc):
    xr,yr=(x+stepsize*np.cos(t+ct),y+stepsize*np.sin(t+ct))
    c2g = euclidean(xr,yr,xg,yg)
    c2c = tc - c2g
    return xr,yr,t+ct,c2g,c2c


