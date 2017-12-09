# PRM-vs-RRT-for-puma560
Final Project for the CS5335: Robotics Science and Systems, Fall'17 course

The project is a comparative analysis between 
Probabilistic Roadmaps (PRM) and Rapidly Exploring 
Random Trees (RRT) for the puma 560 robot.

Authors:
1. Prasanth Murali
2. Himtanaya Bhadada

Requirements:
1. Peter Corke's Robotics Toolbox
2. Matlab_R2017b

Instructions for running the program:
1. Clone the repository and unzip all files.
2. Open rvctools and run the startup_rvc.m program.
3. Open projectStartup.m and run it as 
(projectStartup(50,10)). This will run the PRM program 
for the given workspace. Once the program finishes, one 
can see the trajectory of the robot in moving from start
to goal configurations in the accompanying plot. 
4. To run RRT, in the projectStartup file,
comment out the following lines : 41-42,48-64,66.
Uncomment the following lines: 43-44,73.
This will replace the call to PRM with RRT and the robot 
will do motion planning with RRT algorithm.
