import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
"""
    sim.setStepping() set the simulation mode.
    In this mode, the script has to call sim.step() to
    explicitly run the simulation.
"""
client = RemoteAPIClient()
sim = client.getObject('sim')



sim.startSimulation()
while (t := sim.getSimulationTime()) < 5:
    s = f'Simulation time: {t:.2f} [s]'
    print(s)

sim.stopSimulation()