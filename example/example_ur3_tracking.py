import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from src.ur3 import UR3,PathPlanner
import math

input("""
Before starting, open CoppeliaSim:
  File → Open scene → example/scene/ur3.ttt
Press ENTER to continue...
""")


client = RemoteAPIClient()
sim = client.getObject('sim')

ur3 = UR3(sim,"/UR3")
ur3.reset_target()


quart = [np.sqrt(2)/2,0,np.sqrt(2)/2,0]

circle_y = -0.5
circle_z = 0.4
circle_r = 0.1

thetas = [math.radians(theta) for theta in range(0,360,5)]   

circle_pts = [[-0.8, circle_y+circle_r*math.cos(theta), circle_z+circle_r*math.sin(theta)]
                for theta in thetas]

ctrl_pts = [pt+quart for pt in circle_pts]
ctrl_pts = [ele for sublist in ctrl_pts for ele in sublist]

sim.startSimulation()
path = PathPlanner(sim,ctrl_pts,0.5)
ur3.tracking(path,0.1)

sim.stopSimulation()