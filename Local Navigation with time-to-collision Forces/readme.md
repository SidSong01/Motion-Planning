# This is the implementation of the predictive TTC forces approach for local navigation

.csv files is for the simulation scenarios, classified by its agents numbers

# the format of them:

line 1: the parameters of the first agent:

agent_id, group_id, start_position_x, start_position_y, goal_position_x, goal_position_y, preferred_speed, maximum_speed, radius

line 2: the parameters of the second agent
: : :
line n: the parameters of the nth agent

The approach can be considered as a variant of the PowerLaw model, as both approaches rely on forces that depend on the relative displacement of agents at the moment of a collision.

# Goal force of the agent

![image1](https://github.com/SidSong01/Motion-Planning/blob/master/Local%20Navigation%20with%20time-to-collision%20Forces/goal_force.png)

assume that the goal velocity is the unit vector pointing from the current position of the agent to its goal position scaled by the agent’s preferred speed.

# The force needed for the avoiding collision

![image2](https://github.com/SidSong01/Motion-Planning/blob/master/Local%20Navigation%20with%20time-to-collision%20Forces/repulsive_force.png)

time-to-collision value-t, Typical value for the time horizon t_H is 4s, n is the unit vector that pushes the two agents apart.

# How to use
```sh
$ python simulator.py
``` 

Change the .csv file names to get different agents numbers. tune the parameters for getting best performance. or u can change the function to get better force

[//]: # (Image References)
[image1]: ./example.png

# Results
![alt text][image1]
---
This is the screenshot for the 8 agents scenario.
