# Astar-algorithm-implementation-for-a-non-holonomic-mobile-robot
*Project 3 Phase 1 ENPM661 Path Planning for Autonomous Robots*
## Authors
- [Darshit Desai](https://github.com/darshit-desai); Dir ID: darshit; UID: 118551722
- [Shivam Sehgal](https://github.com/shivamsehgal77); Dir ID: ssehgal7; UID: 119222041

![ezgif com-video-to-gif](https://user-images.githubusercontent.com/36150235/226222995-05983ea1-9c1d-43a7-8b39-09277a598147.gif)

## Code run Instructions

To run the program ensure that you have the following libraries: time, pygame, numpy, queue

Download the code using https://github.com/darshit-desai/Astar-algorithm-implementation-for-a-non-holonomic-mobile-robot.git

For running the code use /bin/python3 /$PATH$/a_star_DarshitMiteshkumar_Shivam.py in the linux or VSCode Terminal

## Results


### Google Drive Links: 
![Video of valid solution](https://drive.google.com/file/d/1VzLooit4g6QmN3XYUQIhVg_RVxbSUGc-/view?usp=sharing),![Video with no solution]( 
https://drive.google.com/file/d/1cQPtLVFklBLRYghyRPMaL3PqzhV6EsEY/view?usp=sharing)

### Here's a screenshot of the explored node vectors (black) and the path taken (green line) with the obstacle map (red):

The parameters of the simulation for search are as below

Terminal inputs:

    ROBOT PARAMETERS
    ***STEP SIZE OF THE ROBOT***
    Enter the step size 
    10
    ROBOT CLEARANCE DIMENSIONS AND RADIUS. Enter valid dimensions between 0 to 50
    Enter the clearance of the robot: 3
    Enter the radius of the robot: 3
    Valid coordinates received
    Enter Robot start and goal coordinates. Ensure that theta values are multiples of 30 deg
    Enter the starting x coordinate: 9
    Enter the starting y coordinate: 20
    Enter the start theta orientation: 0
    Enter the goal x coordinate:300
    Enter the goal y coordinate: 230
    Enter the goal theta orientation: 30
![image](https://user-images.githubusercontent.com/36150235/226221895-8a29293c-3d1f-45f7-80d0-6db00250d0f7.png)

### Animation of the node exploration (black) and the path taken (green line) with the obstacle map (red):

Terminal inputs

        ROBOT PARAMETERS
    ***STEP SIZE OF THE ROBOT***
    Enter the step size
    20
    ROBOT CLEARANCE DIMENSIONS AND RADIUS. Enter valid dimensions between 0 to 50
    Enter the clearance of the robot: 2
    Enter the radius of the robot: 2
    Valid coordinates received
    Enter Robot start and goal coordinates. Ensure that theta values are multiples of 30 deg
    Enter the starting x coordinate: 9
    Enter the starting y coordinate: 9
    Enter the start theta orientation: 30
    Enter the goal x coordinate:575
    Enter the goal y coordinate: 225
    Enter the goal theta orientation: 90



https://user-images.githubusercontent.com/36150235/227049076-11690fb6-3d85-42af-98e3-37499323dd9e.mp4



Terminal outputs: 

    goal reached
    Total time taken for search: 0.7260479927062988
    The final goal node is given by:  13044
    Total time taken for backtracking: 0.0
    ****** The optimum path is **** [(9, 241), (26, 231), (43, 221), (53, 204), (63, 187), (73, 170), (83, 153), (100, 143), (120, 143), (140, 143), (160, 143), (180, 143),            (197, 133), (207, 116), (217, 99), (227, 82), (244, 72), (261, 62), (278, 52), (295, 42), (315, 42), (335, 42), (355, 42), (375, 42), (392, 32), (409, 42), (426, 32),          (443,22), (453, 5), (473, 5), (490, 15), (507, 25), (524, 15), (541, 25), (575, 25)]
    Length of closed nodes= 8550








