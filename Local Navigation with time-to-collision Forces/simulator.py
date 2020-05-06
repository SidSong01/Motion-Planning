# simulator.py



import numpy as np
from tkinter import *
import time
import functools
from agent import Agent

"""
    Initalize parameters to run a simulation
"""
dt = 0.05 # the simulation time step
scenarioFile='8_agents.csv'
doExport = True # export the simulation?
agents = [] # the simulated agents
trajectories = [] # keep track of the agents' traces
ittr = 0 # keep track of simulation iterations 
maxIttr = 500  #how many time steps we want to simulate
globalTime = 0  # simuation time      
reachedGoals = False # have all agents reached their goals

""" 
    Drawing parameters
"""
pixelsize = 1024
framedelay = 30
drawVels = True
QUIT = False
paused = False
step = False
circles = []
velLines = []
gvLines = []

def readScenario(fileName, scalex=1., scaley=1.):
    """
        Read a scenario from a file
    """
    
    fp = open(fileName, 'r')
    lines = fp.readlines()
    fp.close()
    for line in lines:
        agents.append(Agent(line.split(','),0.5,1,10)) # create an agent and add it to the list
    
    # define the boundaries of the environment
    positions = [a.pos for a in agents]
    goals = [a.goal for a in agents]
    x_min =	min(np.amin(np.array(positions)[:,0]), np.amin(np.array(goals)[:,0]))*scalex - 2.
    y_min =	min(np.amin(np.array(positions)[:,1]), np.amin(np.array(goals)[:,1]))*scaley - 2.
    x_max =	max(np.amax(np.array(positions)[:,0]), np.amax(np.array(goals)[:,0]))*scalex + 2.
    y_max =	max(np.amax(np.array(positions)[:,1]), np.amax(np.array(goals)[:,1]))*scaley + 2.

    num = len(agents);

    return x_min, x_max, y_min, y_max 


def initWorld(canvas):
    """
        initialize the agents 
    """
    print ("")
    print ("Simulation of Agents on a 2D plane.")
    print ("Green Arrow is Goal Velocity, Red Arrow is Current Velocity")
    print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print ("")
       
    colors = ["#FAA", "blue","yellow", "white"]
    for a in agents:
        circles.append(canvas.create_oval(0, 0, a.radius, a.radius, fill=colors[a.gid%4])) # color the disc of an agenr based on its group id
        velLines.append(canvas.create_line(0,0,10,10,fill="red"))
        gvLines.append(canvas.create_line(0,0,10,10,fill="green"))
      
def drawWorld():
    """
        draw the agents
    """

    for i in range(len(agents)):
        agent = agents[i]
        if not agent.atGoal:
            canvas.coords(circles[i],world_scale*(agent.pos[0]- agent.radius - world_xmin), world_scale*(agent.pos[1] - agent.radius - world_ymin), world_scale*(agent.pos[0] + agent.radius - world_xmin), world_scale*(agent.pos[1] + agent.radius - world_ymin))
            canvas.coords(velLines[i],world_scale*(agent.pos[0] - world_xmin), world_scale*(agent.pos[1] - world_ymin), world_scale*(agent.pos[0]+ agent.radius*agent.vel[0] - world_xmin), world_scale*(agent.pos[1] + agent.radius*agent.vel[1] - world_ymin))
            canvas.coords(gvLines[i],world_scale*(agent.pos[0] - world_xmin), world_scale*(agent.pos[1] - world_ymin), world_scale*(agent.pos[0]+ agent.radius*agent.gvel[0] - world_xmin), world_scale*(agent.pos[1] + agent.radius*agent.gvel[1] - world_ymin))
            if drawVels:
                canvas.itemconfigure(velLines[i], state="normal")
                canvas.itemconfigure(gvLines[i], state="normal")
            else:
                canvas.itemconfigure(velLines[i], state="hidden")
                canvas.itemconfigure(gvLines[i], state="hidden")

def on_key_press(event):
    """
        keyboard events
    """                    
    global paused, step, QUIT, drawVels

    if event.keysym == "space":
        paused = not paused
    if event.keysym == "s":
        step = True
        paused = False
    if event.keysym == "v":
        drawVels = not drawVels
    if event.keysym == "Escape":
        QUIT = True

def updateSim(dt):
    """
        Update the simulation 
    """

    global reachedGoals
   
    # compute the forces acting on each agent
    for agent in agents:
        agent.computeForces(agents)
    
    
    reachedGoals = True    
    for agent in agents:
        agent.update(dt)
        if not agent.atGoal:
            reachedGoals = False


def drawFrame(dt):
    """
        simulate and draw frames 
    """

    global start_time,step,paused,ittr,globalTime

    if reachedGoals or ittr > maxIttr or QUIT: #Simulation Loop
        print("%s itterations ran ... quitting"%ittr)
        win.destroy()
    else:
        elapsed_time = time.time() - start_time
        start_time = time.time()
        if not paused:
            updateSim(dt)
            ittr += 1
            globalTime += dt
            for agent in agents:
                if not agent.atGoal:
                   trajectories.append([agent.id, agent.gid, agent.pos[0], agent.pos[1], agent.vel[0], agent.vel[1], agent.radius, globalTime])

        drawWorld()
        if step == True:
            step = False
            paused = True    
        
        win.title('Multi-Agent Navigation')
        win.after(framedelay,lambda: drawFrame(dt))
  

#=======================================================================================================================
# Main execution of the code
#=======================================================================================================================
world_xmin, world_xmax, world_ymin, world_ymax = readScenario(scenarioFile)
world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize/world_width

# set the visualizer
win = Tk()
# keyboard interaction
win.bind("<space>",on_key_press)
win.bind("s",on_key_press)
win.bind("<Escape>",on_key_press)
win.bind("v",on_key_press)
# the drawing canvas
canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#666")
canvas.pack()
initWorld(canvas)
start_time = time.time()
# the main loop of the program
win.after(framedelay, lambda: drawFrame(dt))
mainloop()
if doExport:
    header = "id,gid,x,y,v_x,v_y,radius,time"
    exportFile = scenarioFile.split('.csv')[0] + "_sim.csv"
    np.savetxt(exportFile, trajectories, delimiter=",", fmt='%d,%d,%f,%f,%f,%f,%f,%f', header=header, comments='')
