import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from threading import Thread

client = RemoteAPIClient()
sim1 = client.getObject('sim')
client2 = RemoteAPIClient()
sim2 = client2.getObject('sim')

sim1.setStepping(True)
sim2.setStepping(True)


def runSim(numThread,sim,delayTime):
    while (t := sim.getSimulationTime()) < 5:
        sim.step()
        s = f'Simulation time: {t:.2f} [s]'
        print(numThread,s)    
        time.sleep(delayTime)

sim1.startSimulation()

thread1 = Thread(target=runSim,args=[1,sim1,0.001])
thread2 = Thread(target=runSim,args=[2,sim2,0.1])

thread1.start()
thread2.start()

thread1.join()
thread2.join()

sim1.stopSimulation()