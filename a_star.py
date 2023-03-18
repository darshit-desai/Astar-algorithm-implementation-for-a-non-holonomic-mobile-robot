# import the pygame module
import pygame as pyg
import numpy as np
#import queue module
from queue import PriorityQueue
import math
import time


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
white = (255,255,255)

while True:
    try:
        start_x=int(input("Enter the starting x coordinate \n"))    
        start_y=int(input("Enter the starting ycoordinate \n"))    
        goal_x=int(input("Enter the goal coordinate \n"))    
        goal_y=int(input("Enter the goal ycoordinate \n")) 
        step_size=int(input("Enter the step size \n"))
        start_theta=int(input("Enter the start \n"))
        goal_theta=int(input("Enter the goal \n"))
        robot_start_position=(start_x,250-start_y,start_theta)
        robot_goal_position=(goal_x,250-goal_y,goal_theta)
        if screen.get_at(robot_start_position) != white and screen.get_at(robot_goal_position)!=white:
            raise ValueError
        break
    except ValueError:
        print("Wrong input entered. Please enter an integer in correct range x(0,599) and y(0,249).")

def move_robot(x,y,curr_theta,totalcst):
    new_nodes = []
    for t in range(-60,61,30):
        x_t,y_t, t_t, c2g, c2c=  actions(x,y,t,curr_theta,totalcst)
        robot_position=(x_t,y_t,t_t)
        new_nodes.append([c2g+c2c,c2c,c2g,robot_position])
    return new_nodes

def euclidean(x,y,xg,yg):
    return math.dist((x,y),(xg,yg))

def actions(x,y,t,ct,tc):
    xr,yr=(x+step_size*np.cos(t+ct),y+step_size*np.sin(t+ct))
    c2g = euclidean(xr,yr,goal_x,goal_y)
    c2c = tc - c2g
    return xr,yr,t+ct,c2g,c2c

ctc_node=0  # cost to come for start node
ctc_goal=math.dist((start_x,start_y),(goal_x,goal_y))
parent_node_index=None # Index for the parent node
node_index=0 # Index of the current node
closed_list={} # list to store information about the current node
check_closed_list={}
open_list=PriorityQueue() # list the store nodes and pop them according to priority
info=[ctc_goal+ctc_node,ctc_node,ctc_goal,node_index,parent_node_index,robot_start_position] # list to save all info of a node
open_list.put(info)

global_dict={}
global_dict[robot_start_position]=[ctc_goal+ctc_node,ctc_node,ctc_goal,node_index,parent_node_index,robot_start_position]


def new_node(ctc_total,ctc,ctc_g,new_pos):

    if screen.get_at(new_pos) == white and not (new_pos in check_closed_list):
        if not (new_pos in global_dict):
            ctc_node_left = ctc_move + info[0]
            
            global node_index
            node_index += 1
            global_dict[new_pos]=[ctc_node_left, node_index, info[1], new_pos]
            open_list.put(global_dict[new_pos])
        else:
            if (global_dict[new_pos][0]>ctc_move+info[0]):
                global_dict[new_pos][2]=info[1]
                global_dict[new_pos][0]=ctc_move+info[0]

while True:
    # if the open list is empty means that no solution could be found
    if(open_list.empty()):
        print("No solution")
        goal_node=None
        break

    info=open_list.get()

    # if the goal positoin is reached 
    if(info[5]==robot_goal_position):
        print("goalt reached")
        closed_list[info[3]]=[info[0],info[1],info[2],info[4],info[5]]
        goal_node=info[3]
        break

    new_nodes=move_robot()
    for i in range(0,5):
         if(new_nodes[i][3]==robot_goal_position):
              print("goal reached")
              closed_list[node_index+i+1]=






    # # making the node for the new postion and adding to priority queue
    # ctc_move,new_pos=robot.robot_move_left(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)             

    # # making the node for the new postion and adding to priority queue
    # ctc_move,new_pos=robot.robot_move_up(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
    # # making the node for the new postion and adding to priority queue                        
    # ctc_move,new_pos=robot.robot_move_right(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
                            
    # # making the node for the new postion and adding to priority queue  
    # ctc_move,new_pos=robot.robot_move_down(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
    # # making the node for the new postion and adding to priority queue
    # ctc_move,new_pos=robot.robot_move_down_left(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
    # # making the node for the new postion and adding to priority queue
    # ctc_move,new_pos=robot.robot_move_down_right(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
    # # making the node for the new postion and adding to priority queue
    # ctc_move,new_pos=robot.robot_move_up_left(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
    # # making the node for the new postion and adding to priority queue
    # ctc_move,new_pos=robot.robot_move_up_right(info[3])
    # if(new_pos==robot_goal_position):
    #     print("goal reached")
    #     closed_list[node_index+1]=[ctc_move+info[0],info[2],new_pos]
    #     goal_node=node_index+1
    #     break
    # new_node(ctc_move,new_pos)
                            
    # append the node to node list                                               
    closed_list[info[1]]=[info[0],info[2],info[3]]
    check_closed_list[info[3]]=None

    # print(f"Closed_list{info[1]}]",closed_list[info[1]])
    # screen.set_at(info[3], b





























# Set the caption of the screen
pyg.display.set_caption('Djikstra')
pyg.display.update()
running=True
while running:
	# for loop through the event queue
	for event in pyg.event.get():
		# Check for QUIT event	
		if event.type == pyg.QUIT:
			running = False
                        
