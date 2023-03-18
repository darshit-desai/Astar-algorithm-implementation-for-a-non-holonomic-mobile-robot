# import the pygame module
import pygame as pyg
import numpy as np
#import queue module
from queue import PriorityQueue
import math
import time

def custom_round(x):
    int_part, frac_part = divmod(x,1)

    f = frac_part/0.25

    if (f>0 and f<=1):
        return int_part
    elif (f>1 and f<=2):
        return int_part+0.5
    elif (f>2 and f<=3):
        return int_part+0.5
    elif (f>3 and f<=4):
        return int_part+1


while (True):
    clr = int(input("Enter the clearance of the robot: "))
    radii = int(input("Enter the radius of the robot: "))
    if ((clr>0 and clr<50) and (radii>0 and radii<50)):
        print("Valid coordinates received")
        break
    else:
        print("Invalid coordinates received, Try again")


#Define the Surface Map
screen = pyg.display.set_mode((600,250))
#Define the rectangles which make the base map
rect_color = (255, 255, 255)
#Define the rectangle which makes the outer border
rectangle1 = pyg.Rect(clr+radii, clr+radii, 600-2*(clr+radii), 250-2*(clr+radii))
screen.fill((255,0,0))
pyg.draw.rect(screen, rect_color, rectangle1)
#Define the rectangle which makes the 2 rectangles
bottom_rect_dim = [(150+radii+clr,150-radii-clr),(150+radii+clr,250),(100-radii-clr,250),(100-radii-clr,150-radii-clr)]
pyg.draw.polygon(screen, (255,0,0),bottom_rect_dim)
top_rect_dim = [(100-radii-clr,0),(150+radii+clr,0),(150+radii+clr,100+radii+clr),(100-radii-clr,100+radii+clr)]
pyg.draw.polygon(screen,(255,0,0),top_rect_dim)
#Define the hexagon in the center with original dimensions
hexagon_dim = [(300,50-radii-clr),(364.95190528+radii+clr,87.5-((radii+clr)*np.tan(np.pi*30/180))),(364.95190528+radii+clr,162.5+((radii+clr)*np.tan(np.pi*30/180))),(300,200+radii+clr),(235.04809472-radii-clr,162.5+((radii+clr)*np.tan(np.pi*30/180))),(235.04809472-radii-clr,87.5-((radii+clr)*np.tan(np.pi*30/180)))]
# pyg.draw.polygon(screen,(255,0,0),hexagon_dim)
pyg.draw.polygon(screen,(255,0,0),hexagon_dim)
#Define the triangle with the original dimensions
triangle_dim = [(460-radii-clr,25-((radii+clr)/np.tan(np.pi*13.28/180))),(460.00-radii-clr,225+((radii+clr)/np.tan(np.pi*13.28/180))),(510+((radii+clr)/np.cos(np.pi*26.5650518/180)),125)]
# pyg.draw.polygon(screen,(255,0,0),triangle_dim)
pyg.draw.polygon(screen,(255,0,0), triangle_dim)

while True:
    try:
        start_x=int(input("Enter the starting x coordinate: \n"))    
        start_y=int(input("Enter the starting y coordinate: \n")) 
        start_theta=int(input("Enter the start theta: \n"))
        goal_x=int(input("Enter the goal x coordinate: \n"))    
        goal_y=int(input("Enter the goal y coordinate: \n")) 
        goal_theta=int(input("Enter the goal theta: \n"))
        step_size=int(input("Enter the step size: \n"))
        
        start_y=250-start_y
        goal_y=250-goal_y
        if screen.get_at(robot_start_position) != (255,255,255) and screen.get_at(robot_goal_position)!=(255,255,255) and (start_theta%30 != 0 and goal_theta%30 !=0):
            raise ValueError
        break
    except ValueError:
        print("Wrong input entered. Please enter an integer in correct range x(0,599) and y(0,249) and enter theta values in multiples of 30.")
 
cost_to_come = 0
cost_to_goal = math.dist((start_x,start_y),(goal_x,goal_y))
total_cost = cost_to_come+cost_to_goal
start_state = [total_cost,cost_to_goal,cost_to_come, -1,0, (start_x,start_y,start_theta)]
Open_list = PriorityQueue()
Open_list.put(start_state)
Closed_list = {}
visitedNodes3d= np.zeros((500,1200,12),dtype=np.uint16)
while (Open_list.empty()==False):
    nodegenerat = Open_list.get()
    iter_node_tot_cost = nodegenerat[0]
    iter_node_cost_goal = nodegenerat[1]
    iter_node_cost_come = nodegenerat[2]
    iter_node_parent = nodegenerat[3]
    iter_node_index = nodegenerat[4]
    iter_node_state_x, iter_node_state_y, iter_node_state_theta = nodegenerat[-1]
    Closed_list[iter_node_index]=[iter_node_tot_cost,iter_node_cost_goal,iter_node_cost_come,iter_node_parent, (iter_node_state_x, iter_node_state_y, iter_node_state_theta)]
    if ((math.dist((iter_node_state_x,iter_node_state_y),(goal_x,goal_y)))<1.0):
        print("Goal Reached, Backtracking will be done later")
        break
    else:
        new_node_parent_node = iter_node_index
        for t in range (0,121,30):
            new_node_x, new_node_y = (iter_node_state_x+(step_size*np.cos(np.pi*t/180))),(iter_node_state_y+(step_size*np.sin(np.pi*t/180)))
            new_node_theta = iter_node_state_theta+t
            roundnew_node_x = custom_round(new_node_x)
            roundnew_node_y = custom_round(new_node_y)
            roundnew_node_theta = custom_round(new_node_theta)
            if(screen.get_at(new_node_x,new_node_y)!=(255,0,0)):
                if (visitedNodes3d[roundnew_node_y][roundnew_node_x][roundnew_node_theta]==0):
                    visitedNodes3d[roundnew_node_y][roundnew_node_x][roundnew_node_theta]+=1
                    






            





