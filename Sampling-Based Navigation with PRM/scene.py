# scene.py

import time, random
import sys
import tkinter as tk
import tkinter.messagebox
import numpy as np 
from obstacles import BoxObstacle
from utils import *
from graph import *
import copy

class Scene(tk.Frame):
    random.seed('cpsc8810')
   
    def __init__(self, filename, disk_robot, build_fn, master=None, resolution = 700):
        super().__init__(master)
        #self.master = tk.Tk()
        self.build_fn = build_fn
        self.filename = filename
        self.resolution = resolution
        self.disk_robot = disk_robot 

        # parameters related to the problem
        self.scene_width, self.scene_height = None, None
        self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax = None, None, None, None
        self.default_start, self.default_goal = None, None
        self.start, self.goal = None, None
        self.obstacles = None
        self.robot_width, self.robot_height = None, None
        self.roadmap = None
        if not self.loadProblem(): sys.exit("Failed to load problem file")

        # setup the GUI
        self.master.title("PRM Planner")        
        self.canvas = tk.Canvas(self.master, width=resolution, height=resolution, bg="white")
        #self.canvas.pack()
        self.bt_new = tk.Button(self.master, text="Random Query", command=self.random_query, state=tk.DISABLED)
        self.bt_default = tk.Button(self.master, text="Default Query", command=self.default_query, state=tk.DISABLED)
        self.bt_search = tk.Button(self.master, text="Get Path", command=self.search, state=tk.DISABLED)
        self.bt_roadmap = tk.Button(self.master, text="Generate Roadmap", command=self.generate)
        self.master.resizable(False, False)

        self.canvas.grid(row=0, columnspan=4,
            sticky=tk.W+tk.E+tk.N+tk.S, padx=10, pady=28
        )
        self.bt_new.grid(row=1, column=0,
            sticky=tk.W, padx=10, pady=(0, 10)
        )
        self.bt_default.grid(row=1, column=1,
            sticky=tk.W, padx=5, pady=(0, 10)
        )
        self.bt_search.grid(row=1, column=2,
            sticky=tk.W, padx=10, pady=(0, 10)
        )
        self.bt_roadmap.grid(row=1, column=3,
            sticky=tk.E, padx=10, pady=(0, 10)
        )
        
        self.master.columnconfigure(0, weight=0)
        self.master.columnconfigure(1, weight=0)
        self.master.columnconfigure(2, weight=0)
        self.master.columnconfigure(3, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.master.rowconfigure(1, weight=0)
       
        self.draw_scene()
        #path = self.interpolate(self.default_start,self.default_goal,2)
     
        #self.canvas.bind("<Configure>", self.default_query)

    def loadProblem(self):
        """
            Read a scenario from a file
        """
        try:
            fp = open(self.filename, 'r')
            lines = fp.readlines()
            fp.close()
    
            # first line reads the dimension 
            # second line reads the dimension of the robot 
            scene_parameters = lines[0].split(',')
            robot_parameters = lines[1].split(',')
            query_parameters = lines[2].split(',')
            self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax = int(scene_parameters[0]), int(scene_parameters[1]), int(scene_parameters[2]), int(scene_parameters[3])
            self.scene_width = self.scene_xmax -self.scene_xmin 
            self.scene_height = self.scene_ymax -self.scene_ymin 
            self.robot_width = float(robot_parameters[0])
            self.robot_height = float(robot_parameters[1])
            self.default_start = (float(query_parameters[0]), -float(query_parameters[1]), float(query_parameters[2]))
            self.default_goal = (float(query_parameters[3]), -float(query_parameters[4]), float(query_parameters[5]))
                        
        
            self.obstacles = []
            for line in lines[3:]:
                parameters = line.split(',')
                vertices = []
                vertices.append((float(parameters[0]), -float(parameters[1]))) #y-axis is flipped
                vertices.append((float(parameters[2]), -float(parameters[3])))
                vertices.append((float(parameters[4]), -float(parameters[5])))
                vertices.append((float(parameters[6]), -float(parameters[7])))
                self.obstacles.append(BoxObstacle(vertices))
        except:
            return False
        
        return True


    def getObstacles(self):
        return self.obstacles
    
    def getRobot(self):
        return self.robot_width, self.robot_height            
    
    def default_query(self, event=None):
        self.bt_default.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_roadmap.config(state=tk.DISABLED)
        self.bt_search.config(state=tk.DISABLED)

 
        self.canvas.delete("start")
        self.canvas.delete("goal")
        self.canvas.delete("path")
        self.start = self.default_start
        self.goal = self.default_goal
        self.draw_config(self.start,"green","start")
        self.draw_config(self.goal,"blue","goal")
      
        self.bt_default.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        self.bt_roadmap.config(state=tk.NORMAL)
        self.bt_search.config(state=tk.NORMAL)


    def random_query(self, event=None):
        self.bt_default.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_roadmap.config(state=tk.DISABLED)
        self.bt_search.config(state=tk.DISABLED)
 
        self.canvas.delete("start")
        self.canvas.delete("goal")
        self.canvas.delete("path")
        self.start = (random.uniform(self.scene_xmin, self.scene_xmax), random.uniform(self.scene_ymin, self.scene_ymax), random.uniform(0, np.pi))
        self.goal = (random.uniform(self.scene_xmin, self.scene_xmax), random.uniform(self.scene_ymin, self.scene_ymax), random.uniform(0, np.pi))
        #self.draw_config(self.start,"#43a2ca","start")
        #self.draw_config(self.goal,"#e0f3db","goal")
        self.draw_config(self.start,"green","start")
        self.draw_config(self.goal,"blue","goal")
      
        self.bt_default.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        self.bt_roadmap.config(state=tk.NORMAL)
        self.bt_search.config(state=tk.NORMAL)
        
    def search(self):
        self.bt_search.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_default.config(state=tk.DISABLED)
        self.bt_roadmap.config(state=tk.DISABLED)
      
     
        # should return a list of configurations
        roadmap = copy.deepcopy(self.roadmap)
        p = self.build_fn[1](self.start, self.goal, roadmap)

        if p is None or len(p) == 0:
            tk.messagebox.showinfo("", "Failed to find any solution path.")
        else:
            self.draw_path(p)

       

        self.bt_search.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        self.bt_default.config(state=tk.NORMAL)
        self.bt_roadmap.config(state=tk.NORMAL)
    
    def generate(self):
        self.clear_canvas()
        self.bt_roadmap.config(state=tk.DISABLED)
        self.bt_search.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_default.config(state=tk.DISABLED)
     
        # should return a graph
        self.roadmap = self.build_fn[0]([(self.scene_xmin, self.scene_xmax),
        (self.scene_ymin, self.scene_ymax), (0,2*np.pi)], (self.robot_width, self.robot_height), self.obstacles)

        # should draw the graph 
        if self.roadmap is None or len(self.roadmap.getVertices()) == 0:
            tk.messagebox.showinfo("", "Failed to construct a roadmap.")
        else:
            self.draw_roadmap(self.roadmap, 0.5)

       
        #self.bt_search.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        self.bt_default.config(state=tk.NORMAL)
        self.bt_roadmap.config(state=tk.NORMAL)


    def draw_scene(self):
        self.clear_canvas()
        world_scale = self.resolution/self.scene_width
        for obst in self.obstacles:
            self.canvas.create_rectangle(world_scale*(obst.x_min - self.scene_xmin), world_scale*(obst.y_min - self.scene_ymin), 
            world_scale*(obst.x_max - self.scene_xmin),  world_scale*(obst.y_max - self.scene_ymin), fill="red", tag="obstacle")
        
       
    def draw_config(self, config, color, name):
        world_scale = self.resolution/self.scene_width
        if self.disk_robot: 
            radius = self.robot_width/2
            u_x = config[0] - self.scene_xmin
            u_y = config[1] - self.scene_ymin
            self.canvas.create_oval(world_scale*(u_x -radius),world_scale*(u_y - radius), 
                world_scale*(u_x + radius), world_scale*(u_y + radius), fill=color, tag= name)

        else:
            points = getRobotPlacement(config, self.robot_width, self.robot_height)
            corners = [(world_scale*(x[0]-self.scene_xmin), world_scale*(x[1] - self.scene_ymin))  for x in points]

            self.canvas.create_polygon(corners,
                    fill=color, tag= name
            )

         
    def draw_path(self, path):
        #start_color = np.array([227, 74, 51])
        #end_color =  np.array([254, 232, 200])
        #start_color = np.array([67,162,202])
        #end_color =  np.array([224,255,219])
        start_color = np.array([0,255,0])
        end_color =  np.array([0,0,255])
        for i in range(len(path)):
            color = start_color + float(i)/float(len(path)) * (end_color - start_color)
            tk_rgb = "#%02x%02x%02x" % tuple(int(c) for c in color)
            self.draw_config(path[i], tk_rgb, "path")
          
    
    def draw_roadmap(self, roadmap, radius=1.0):
        world_scale = self.resolution/self.scene_width
        for i,u in enumerate(roadmap.getVertices()):
            u_x = u.getConfiguration()[0] - self.scene_xmin
            u_y = u.getConfiguration()[1] - self.scene_ymin
            for e in u.getEdges():   # draw edges
                v = roadmap.getVertices()[e.getId()]
                self.canvas.create_line(world_scale*u_x, world_scale*u_y, world_scale*(v.getConfiguration()[0] - self.scene_xmin), 
                world_scale*(v.getConfiguration()[1] - self.scene_ymin), fill="grey40", dash = (1,1), tag="roadmap") 
            self.canvas.create_oval(world_scale*(u_x -radius), world_scale*(u_y -radius), world_scale*(u_x + radius), 
            world_scale*(u_y + radius), fill="black", tag="roadmap")    #draw vertex
             

    def clear_canvas(self):
        self.canvas.delete("roadmap")
        self.canvas.delete("start")
        self.canvas.delete("goal")
        self.canvas.delete("path")
        #self.canvas.delete("obstacle")
      