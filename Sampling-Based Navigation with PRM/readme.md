# This is the implementation of the probabilistic roadmap method

Motion planning queries for a 2D robot moving in an environment filled with static obstacles.

[//]: # (Image References)
[image1]: ./example.png

## The parameters of PRM:

amount of samples:800

maximal distance between each sample: 2

stepsize for checking interpolate: 1/15

maximal distance for add neighbors of one configuration: 2.2

amount of k_nearest_neighbors: 4


## Hwo to run:
```sh
$ python prmplanner_v1.1.py
``` 

## Outcome

![alt text][image1]
---
Black points are the sampled vertexes, dashed lines are the edges. Red parts are the obstacles, and blue and green cicles represent the final path.
