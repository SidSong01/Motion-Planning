# agent.py

 
import numpy as np
from math import sqrt
import random


class Agent(object):
    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=4, goalRadiusSq=1, maxF = 10):
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
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent

    # compute time to collide
    def isotropic_ttc(self, agent):
        """
        Function to compute the time-to-collision
        between two agents agents, using the candidate velocities
        
        """
        eps = 0.2
        #vi = 0.1
        #h = random.uniform(0,vi)
        r = self.radius + agent.radius
        x = self.pos - agent.pos
        c = np.dot(x, x) - r * r
        if c < 0:   # agents are colliding
            return 0
        #v = self.vel - agent.vel + h
        v = self.vel - agent.vel
        a = np.dot(v, v) - eps*eps
        b = np.dot(x, v) - eps*r
        if b > 0:   # agents are moving away
            return np.inf
        discr = b * b - a * c
        if discr <= 0:
            return np.inf
        tau = c / (-b + sqrt(discr))
        if tau < 0:
            return np.inf
        return tau

    def computeForces(self, neighbors=[]):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """
        if not self.atGoal:
            closeneighbors = []
            for otherAgent in neighbors:
                if otherAgent.id != self.id:
                    otherAgent.tmp_dist = np.linalg.norm(np.subtract(self.pos, otherAgent.pos)) - self.radius - otherAgent.radius
                    if otherAgent.tmp_dist <= self.dhor:
                        closeneighbors.append(otherAgent)     # to add the agent into the neighbor list
            if closeneighbors is None:
                self.F = np.zeros(2)
                return self.F
            
            else:
                closeneighbors.sort(key = lambda x:x.tmp_dist) # sort the closeagents for considering the nearest ones
                
                # goal force
                self.F = (self.gvel - self.vel) / self.ksi
                # avoidance force
                Favoid = np.zeros(2)
                for neighbor in neighbors:
                    tau = self.isotropic_ttc(neighbor)
                    if tau is np.inf:
                        favoid = np.zeros(2)
                    else:
                        dir = self.pos + self.vel * tau - neighbor.pos - neighbor.vel * tau
                        if tau is 0:
                            tau = np.finfo('float64').eps
                        dir_norm = max(np.linalg.norm(dir, ord=1), np.finfo(dir.dtype).eps)
                        n = dir / dir_norm
                        favoid = (max(self.timehor - tau, 0) / tau) * n
                    Favoid = Favoid + favoid
                self.F = self.F + Favoid

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            if np.linalg.norm(self.F) > self.maxF:
                self.F = self.maxF * (self.F / np.linalg.norm(self.F))  #cap the maximum force
            self.vel += self.F*dt     # update the velocity
            if np.linalg.norm(self.vel) > np.linalg.norm(self.maxspeed):
                self.vel = self.maxspeed                                # cap the maximum speed
            self.pos += self.vel*dt   # update the position
        
            # compute the goal velocity for the next time step. do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed