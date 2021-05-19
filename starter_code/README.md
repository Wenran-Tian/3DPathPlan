# ECE276B PR2 SP20 

## Overview
In this assignment, you will implement and compare the performance of search-based and sampling-based motion planning algorithms on several 3-D environments.

### 1. main.py
This file contains examples of how to load and display the 7 environments and how to call a motion planner and plot the planned path. Feel free to modify this file to fit your specific goals for the project. In particular, you should certainly replace Line 104 with a call to a function which checks whether the planned path intersects the boundary or any of the blocks in the environment.

### 2. Planner.py
This file contains an implementation of a baseline planner. The baseline planner gets stuck in complex environments and is not very careful with collision checking. Feel free to modify this file in any way necessary for your own implementation.

### 3. maps
This folder contains the 7 test environments described via a rectangular outer boundary and a list of rectangular obstacles. The start and goal points for each environment are specified in main.py.


