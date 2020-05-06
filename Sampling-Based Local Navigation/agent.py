# agent.py



import numpy as np
from math import sqrt, pi
import random



class Agent(object):

    def __init__(self, csvParameters, dhor = 5, goalRadiusSq=1):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent     
        self.tmp_dist = 0
        
    def computeTau(self,v_cand,other):

        """
        Function to compute the time-to-collision
        between two agents agents, using the candidate velocities
        
        """
        
        rad = self.radius + other.radius
        w = self.pos - other.pos
        c = w.dot(w) - rad * rad
        if c < 0:
            return 0
        v = v_cand - other.vel
        a = v.dot(v)
        b = w.dot(v)
        if b > 0:
            return float('inf')
        discr = b*b - a*c
        if discr <= 0:
            return float('inf')
        tau = c/(-b + sqrt(discr))
        if tau < 0:
            return float('inf')
        return tau
    
  
    def computeNewVelocity(self, neighbors=[]):      
        """
        cost functions can be switched between the functions from the homewor or the pare
        the parameters can be tuned according the performance of the simulation
        k nearest neighbors are taking into calculation through the list after being sorted
        I tried to earn those extra credits
        
        """
        distGoal = np.linalg.norm(self.pos - self.goal)
        if distGoal > self.goalRadiusSq:         # to judge if the agent is near their goals
            closeneighbors = []
            for otherAgent in neighbors:
                if otherAgent.id != self.id:
                    otherAgent.tmp_dist = np.linalg.norm(np.subtract(self.pos, otherAgent.pos)) - self.radius - otherAgent.radius
                    if(otherAgent.tmp_dist <= self.dhor):
                        closeneighbors.append(otherAgent)     # to add the agent into the neighbor list
                   
            if closeneighbors is None:
                self.vnew[:] = self.gvel[:]
                return self.vnew
            else:
                closeneighbors.sort(key = lambda x:x.tmp_dist)     # sort the list with the distance
                af = 0.9
                bt = 1
                gm = 3
                w = 32
                N = 150
                f_min = np.inf
                for i in range(N):
                    velradius = random.uniform(0,self.maxspeed);
                    angle = random.uniform(0,2*pi)
                    vx = velradius * np.cos(angle)
                    vy = velradius * np.sin(angle)
                    tauc = np.inf
                    for neighbor in closeneighbors[0:min(len(closeneighbors),4)]:   # just consider k nearest neighbors
                        t = self.computeTau([vx,vy],neighbor)
                        tauc = max(min(tauc,t),np.finfo(np.float32).eps)
                    #f = w * (1/(tauc)) + np.linalg.norm(np.subtract(self.gvel,[vx,vy]))
                    f = af * np.linalg.norm(([vx,vy] - self.gvel)) + bt * np.linalg.norm(([vx,vy] - self.vel)) + (gm/tauc)
                    if f < f_min:
                        f_min = f
                        self.vnew[:] = [vx,vy]
        else:
            self.atGoal = True
        return self.vnew
   
    
    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        self.goalRadiusSq = 1
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
            
            
  