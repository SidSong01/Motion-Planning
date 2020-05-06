# prmplanner.py



from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
from math import sqrt
import numpy as np

disk_robot = True #(change this to False for the advanced extension) 


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# Construction phase: Build the roadmap


def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable
    
# get the vertices    
    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    
    robot_radius = max(robot_width, robot_height)/2.
    

    # the roadmap 
    graph = Roadmap()

    nr = 800 # used for the amount of samples    
    while graph.getNrVertices() < nr:
        for i in range(nr):
            addVertice = True
            q_new = [np.random.randint(x_limit[0],x_limit[1]),np.random.randint(y_limit[0],y_limit[1])]
            for vertice in graph.getVertices():
                if distance(vertice.getConfiguration(),q_new) < 2:
                    addVertice = False      
            if addVertice and not collision(q_new):
                graph.addVertex(q_new)

# get the edges 
    stepsize = 1/15 # used for checking the interpolate
    path = []
    Vertices = graph.getVertices()
    for v in Vertices:
        neighbor, neighbor_d = nearest_neighbors(graph, v.q, max_dist=2.2)
        if len(neighbor) < 4:
            neighbor, neighbor_d = k_nearest_neighbors(graph, v.getConfiguration(), K=4)
        for neigh in neighbor:
            if not interpolate(v.getConfiguration(), neigh.getConfiguration(), stepsize):
                graph.addEdge(v,neigh,neighbor_d[neighbor.index(neigh)],path)


    #uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph



# Query phase: Connect start and goal to roadmap and find a path using A*

    
def find_path(q_start, q_goal, graph):
    path  = [] 
    
     # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      
    
    start_n = graph.addVertex([q_start[0],q_start[1]])
    end_n = graph.addVertex([q_goal[0],q_goal[1]])
    
    Vertices = graph.getVertices()
    
    parent = ["" for i in range(len(Vertices))]
    
# add the start and goal into the roadmap
    distances = np.inf
    distanceg = np.inf
    
    for v in Vertices:
        distancec_s = distance(v.q,start_n.q)
        if v.id != start_n.id and distancec_s < distances:
            min_s = distancec_s
            distances = distancec_s
            near_s = v
    
    for a in Vertices:
        distancec_g = distance(a.q,end_n.q)
        if a.id != end_n.id and distancec_g < distanceg:
            min_g = distancec_g
            distanceg = distancec_g
            near_g = a
    
    graph.addEdge(start_n,near_s,min_s)
    graph.addEdge(end_n,near_g,min_g)
    
 
# performa a star
    v_list = graph.getVertices()
    nr = graph.getNrVertices()
    #the heuristic
    heuristic = []
    for v in v_list:
        dis = distance(v.q, [q_goal[0],q_goal[1]])
        heuristic.append(dis)

    start = v_list[nr-2]

    h = heuristic[start.id]
    g = 0
    f = g+h
    open_set.put(start, Value(f=f,g=g))

    while len(open_set) > 0:
        q, v = open_set.pop()
        closed_set.add(q)
        g = v.g
        if q.getConfiguration() == [q_goal[0],q_goal[1]]:
            break
        for e in q.getEdges():
            ind = e.getDestination()
            ver_can = v_list[ind]
            if ver_can not in closed_set:
                g2 = g + e.getDist()
                if ver_can not in open_set or open_set.get(ver_can).g > g2:
                    f2 = g2 + heuristic[ind]
                    parent[ind] = e.getSource()
                    open_set.put(ver_can, Value(f=f2, g=g2))         
    
    if(q.getConfiguration() != [q_goal[0],q_goal[1]]):
        path = []
    else:
        while(q.getConfiguration() != [q_start[0],q_start[1]]):
            q = v_list[parent[q.getId()]]  
            path.insert(0,q.getConfiguration())

    return path   


# ----------------------------------------


def nearest_neighbors(graph, q, max_dist):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """
    neigh = []
    neigh_dis = []
    vertices = graph.getVertices()
    
    for v in vertices:
        if v.getConfiguration()!=q and distance(q,v.getConfiguration()) < max_dist:
            neigh.append(v)
            neigh_dis.append(distance(q,v.getConfiguration()))
            

    return neigh, neigh_dis


def k_nearest_neighbors(graph, q, K):
    """distance()
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances
    """
    q_neigh_k = []
    q_neigh_k_d = []
    s = PriorityQueue(order=min)
    vertices = graph.getVertices()
    for v in vertices:
        if v.getConfiguration()!=q:
            s.put(v, distance(q,v.getConfiguration()))
    for i in range(K):
        v, dis = s.pop()
        q_neigh_k.append(v)
        q_neigh_k_d.append(dis)
        
    return q_neigh_k,q_neigh_k_d

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """
    dis = sqrt((q1[0] - q2[0])*(q1[0] - q2[0]) + (q1[1] - q2[1])*(q1[1] - q2[1]))

    return dis

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.  
    """
    col = False
    for obst in obstacles:
        if (q[0] > obst.x_min - robot_radius) and (q[0] < obst.x_max + robot_radius):
            if (q[1] > obst.y_min - robot_radius) and (q[1] < obst.y_max + robot_radius):
                col = True
#    col = False
#    for i in range(len(Scene.obstacles)):
#        for j in range(4):
#            if q[0] > Scene.obstacles[j].x_min && q.[0] < Scene.obstacles[j].x_max && q[1] > Scene.obstacles[j].y_min && q[1] < Scene.obstacles[j].y_max
#            col = True
    return col 
   

def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    
    N = int(1/stepsize)
    dx = (q2[0] - q1[0])/N
    dy = (q2[1] - q1[1])/N
    for i in range(N):
        q_temp = [q1[0] + (i+1) * dx, q1[1] + (i+1) * dy]
        inter = collision(q_temp)
        if inter:
            return True
    return False
#    dis_temp = distance (q1, q2)
#    for i in range(len(dis_temp // stepsize)):
#        tan = (q2[1]-q1[1])/(q2[0]-q1[0])
#        d_x = stepsize * sqrt(1/(tan^2 + 1))
#        d_y = d_x * tan
#        q_temp = [q1[0] + d_x, q1[1] + d_y]
#        
#        if (q_temp[0] < q2[0]) and (q_temp[1] < q2[1]):
#            inter = collision(q_temp)
#            if inter:
#                return inter
            


if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
