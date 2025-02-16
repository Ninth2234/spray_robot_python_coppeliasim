import time
from src.spray_gun import SprayGun
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


input("""
Before starting, open CoppeliaSim:
  File → Open scene → example/scene/spray.ttt
Press ENTER to continue...
""")


client = RemoteAPIClient()
sim = client.getObject('sim')

spray_gun = SprayGun(sim,"/PaintNozzle")

for angle in range(15,60,1):
    spray_gun.set_spray_cone(angle)
    time.sleep(0.1)


print("SIMULATION START")
sim.startSimulation()

spray_gun.on()
for angle in range(60,15,-2):
    spray_gun.set_spray_cone(angle)
    time.sleep(0.1)

for angle in range(15,60,2):
    spray_gun.set_spray_cone(angle)
    time.sleep(0.1)

print("SIMULATION STOP")
sim.stopSimulation()

