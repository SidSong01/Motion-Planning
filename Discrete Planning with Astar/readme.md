# This is the implementation of A* to plan the discrete motion of a simple car

[//]: # (Image References)
[image1]: ./example.png

# How to use

```sh
$ python astar.py
```

The state space is 3-dimensional and is given by an x position, a y position, and an orientation theta. see the example for reference.

the matrix means a simple map with '-' are obstacles, for the final output: F-forward, R-turn right, L-turn left, '*'-the goal. Expanded Nodes means the x,y,theta during along path to the goal.

change the map or cost of different actions for differnet testing

![alt text][image1]
