# import the pygame module
import pygame as pyg
import numpy as np
#import queue module
from queue import PriorityQueue
import time


while (True):
    clr = int(input("Enter the clearance of the robot: "))
    radii = int(input("Enter the radius of the robot: "))
    if ((clr>0 and clr<105) and (radii>0 and radii<105)):
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
pyg.draw.aaline(screen,(0,0,0),(6,6),(90,90))

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