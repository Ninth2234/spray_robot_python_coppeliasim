import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from src import spray_gun
from src.ur3 import UR3,PathPlanner
from src.spray_gun import SprayGun


client = RemoteAPIClient()
sim = client.getObject('sim')

spray = SprayGun(sim,"/PaintNozzle")
spray.clear_spray()
spray.set_spray_cone(15)

ur3 = UR3(sim,"/UR3")
ur3.reset_target()


quart = [np.sqrt(2)/2,0,np.sqrt(2)/2,0]

x = -0.3
y0 = -0.1
y1 = 0.2
z0 = 0.3  
z1 = 0.6
dz = 0.1


ctrl_pts = []
for z in np.arange(z0,z1,dz):
    ctrl_pts.append([x,y0,z]+quart)
    ctrl_pts.append([x,y1,z]+quart)
    ctrl_pts.append([x,y1,z+dz/2]+quart)
    ctrl_pts.append([x,y0,z+dz/2]+quart)

ctrl_pts_1D = [element for sublist in ctrl_pts for element in sublist]

sim.startSimulation()

path = PathPlanner(sim,ctrl_pts_1D,0)

ur3.move_pose(path.get_interpolate_pose(0),[0.1,0.1,0.1,0.1])

spray.on()
ur3.tracking(path,0.1)
spray.off()

sim.stopSimulation()

time.sleep(0.1)
spray.clear_spray()