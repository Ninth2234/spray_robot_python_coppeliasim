from src.utils import read_param,write_param
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


SIM_DRAWING_TRIANGLE_POINTS = 9

default_spray_config = {    
    'jet_angle': 30*3.14/180,  # Convert degrees to radians   
    'jet_range': 0.4, ## m
    'mode': SIM_DRAWING_TRIANGLE_POINTS,  # Same mode as Lua
    'color': [1, 0, 0],  # Convert Lua table to Python list
    'item_size': 3,
    'buffer_size': 300000,
    'density': 64,
    'clear_at_end': False,
    'painting_enabled': False
}
class SprayGun:
    def __init__(self, sim, name):
        self.sim = sim
        self.name = name
        self.spray_config = default_spray_config
        self.objh = sim.getObject(self.name)

        self._write_param(default_spray_config)

    def _write_param(self,data):
        write_param(self.sim,self.objh,"spray",data)

    def _read_param(self):
        return read_param(self.sim,self.objh,"spray")
    
    def on(self):
        self._write_param({'painting_enabled':True})

    def off(self):
        self._write_param({'painting_enabled':False})

    def set_spray_cone(self,angle_deg):
        self._write_param({'jet_angle':angle_deg*3.14/180})

    def spray_color(self, color):
        self._write_param({'color':color})

    def clear_spray(self):
        scriptHandle = self.sim.getObject(self.name+"/script")
        self.sim.callScriptFunction("removeAllDrawingObjects",scriptHandle)

    def get_spray_obj(self,objh):
        scriptHandle = self.sim.getObject(self.name+"/script")
        return self.sim.callScriptFunction("getDrawingObject",scriptHandle,objh)
    


if __name__ == "__main__":
    
    from time import sleep

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    spray_gun = SprayGun(sim,"/PaintNozzle")
    
    sim.startSimulation()
    spray_gun.on()
    sleep(1)
    spray_gun.off()
    print(spray_gun.get_spray_obj(sim.getObject("/Cuboid")))
    sim.stopSimulation()
    # spray_gun.clear_spray()