# This is the implementation of sampling-based velocity approach for local navigation

.csv files is for the simulation scenarios, classified by its agents numbers

# The format of them:

line 1: the parameters of the first agent:

agent_id, group_id, start_position_x, start_position_y, goal_position_x, goal_position_y, preferred_speed, maximum_speed, radius

line 2: the parameters of the second agent
: : :
line n: the parameters of the nth agent

# The cost function used in this project:

![image](https://github.com/SidSong01/Motion-Planning/blob/master/Sampling-Based%20Local%20Navigation/cost_function.png)

Here, Alpha, Beta, and Gamma are the scaling constants that control the relative importance of the three cost terms.

tc denotes the minimum time that it will take for the agent to collide with any of its sensed neighbors if it moves at velocity v_cand. v_goal is the goal velocity.

# How to use
```sh
$ python simulator.py
``` 

Change the .csv file names to get different agents numbers. tune the parameters for getting best performance.

[//]: # (Image References)
[image1]: ./example.png

# Result
![alt text][image1]
---
Screenshot of running with crossing agents.
