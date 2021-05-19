# Path planer in 3D
In this project, we will talk about the possible solutions for motion planning for a point in 3D environment which will contain a series of AABB blocks, and a start and a goal. 

Firstly, we try to find the solution by a weighted A* algorithm, which is search-based, and a famous sample-based algorithm which is called RRT*. 

Secondly, we will compare the performance for these two algorithms and try to give some basic concepts about how to choose them in real implementation. 

We implemented a A* alg, and used the RRTStar class from python motion planning library

The all the main contents are in starter_code file:

## main.py
This is the main file to do the real tests, we can change the algorithm at about line 91

## collsion_check.py
This will provide a class called "collsion_check", which can help us to check if there is any collision between a line and any block in environment

## wAStar.py
This will provide a class to search the path which is based on weighted A* algorithm.

## RRTS.py
This will provide a class to search the path which is based on RRT* algorithm. The class seals the RRTStar class from python motion planning library.

## parameters.py

The file gives all necessary system parameters, and the control input and cost in A* algorithm.