import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from src import spray_gun
from src.ur3 import UR3,PathPlanner
from src.spray_gun import SprayGun
import threading

client = RemoteAPIClient()
sim = client.getObject('sim')

path_ctrl_pts = [
    0.0, 0.0, 0.4,  0, 0, 0, 1,  # Start point (identity rotation)
    0.2, 0.1, 0.4,  0, 0, 0, 1,  # Slight movement forward
    0.4, 0.2, 0.5,  0, 0, 0.707, 0.707,  # Small height change with slight rotation
    0.6, 0.3, 0.6,  0, 0, 1, 0,  # More height and rotation around z-axis
    0.8, 0.4, 0.4,  0, 0, 0.707, -0.707,  # Lowering with inverse rotation
    1.0, 0.5, 0.4,  0, 0, 0, 1  # Back to flat orientation
]
path_smoothness = 1

def _create_spray_control_pts():
    x = 0.2
    z0 = -0.1
    z1 = 0.1
    y0 = -0.15  
    y1 = 0.2
    dy = 0.1

    quart = [0.7071,0,0.7071,0]
    ctrl_pts = []

    for y in np.arange(y0, y1, dy):
        ctrl_pts.append([x, y, z0] + quart)
        ctrl_pts.append([x, y, z1] + quart)
        ctrl_pts.append([x, y + dy / 2, z1] + quart)
        ctrl_pts.append([x, y + dy / 2, z0] + quart)

    ctrl_pts_1D = [element for sublist in ctrl_pts for element in sublist]
    return ctrl_pts_1D

# path = PathPlanner(sim,path_ctrl_pts,path_smoothness)

path = PathPlanner(sim,[],create_new_path=False,exist_path_handle=sim.getObject("/Conveyor_Path"))

sim.stopSimulation()
sim.startSimulation()

sample_body = sim.getObject('/sample_body')
# new_body = sim.copyPasteObjects([sample_body])[0]
# sim.setObjectAlias(new_body,"new_body")
# dummy = sim.createDummy(0.1)
# sim.setObjectParent(new_body, dummy, False)
# sim.setObjectPose(new_body,[0,0,-0.1,0,0,0,1],dummy)
# # pose = sim.getObjectPose(new_body)

# shift_pose = sim.buildPose([0,0,-0.1],[0,0,0])



class Conveyor:

    before_create_callbacks = []
    before_delete_callbacks = []

    def __init__(self, sim, conveyor_path:PathPlanner, attached_obj_handle, vel=0.025):
        self.sim = sim
        self.path = conveyor_path
        self.template_objh = attached_obj_handle

        self.attach_objhs = dict() # (obj_handle: distance)
        
        self.vel = vel
        self.interval = 0.7
        self.part_xyzeul = [0,0,-0.4,0,-1.57,0]
        self.part_shift_weight = [0.05,0.05,0.05,0.2,0.2,0.2]

        self.running = True
        self.lock = threading.Lock()


    def set_part_pose(self, xyz, eul):
        self.shift_pose = sim.buildPose(xyz,eul)

    @classmethod
    def on_create_part(cls, func):
        cls.before_create_callbacks.append(func)
        return func
        

    @classmethod
    def on_delete_part(cls, func):
        cls.before_delete_callbacks.append(func)
        return func
          

    def _setup(self):
        ...

    def _create_new_object(self):

        # run callback
        for callback in self.before_create_callbacks:
            callback(self)

        # calculate pose of object
        xyzeul = np.array(self.part_xyzeul) + np.array(self.part_shift_weight)*np.random.uniform(-1, 1, size=[1,6]).flatten()
        xyzeul = list(xyzeul)
        shift_pose = self.sim.buildPose(xyzeul[0:3],xyzeul[3:6])

        dummy = self.sim.createDummy(0.05)
        self.sim.setObjectPose(dummy,self.path.get_interpolate_pose(0))
        obj = self.sim.copyPasteObjects([self.template_objh])[0]
        self.sim.setObjectPose(obj,shift_pose,dummy)
        self.sim.setObjectAlias(dummy,"hanger")
        self.sim.setObjectParent(obj, dummy)
        self.attach_objhs[dummy] = 0

    def _delete_object(self, attached_objh):
        for callback in self.before_delete_callbacks:
            callback(self,attached_objh)
        
        
        child_objh = sim.getObjectChild(attached_objh, 0)
        
        self.sim.removeObjects([child_objh,attached_objh])

        
    def force_move(self, distance):
        
        with self.lock:
            time.sleep(0.05)
            self._move(distance/self.vel)
            

    def _move(self, dt):
        delete_obj = []
        
        for objh,distance in self.attach_objhs.items():
            
            distance += self.vel*dt
            
            if distance > self.path.get_length():
                self._delete_object(objh)
                delete_obj.append(objh)
                continue
            new_pose = self.path.get_interpolate_pose(distance)
            self.sim.setObjectPose(objh,new_pose)
            
            self.attach_objhs[objh] = distance
        
        for obj in delete_obj:
            self.attach_objhs.pop(obj)
    
    def set_speed(self, vel):
        self.vel = vel

    def _run(self):
        i = 0
        t_past = 0
        self._create_new_object()
        while self.running:
            t = self.sim.getSimulationTime()

            with self.lock:
                self._move(t - t_past)
            
            if t // (self.interval/self.vel) > i:
                i = t // (self.interval/self.vel)
                self._create_new_object()
            
            t_past = t

    def stop(self):
        self.running = False
        self.thread.join()

    def start(self):
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

class ProximitySensor:

    _on_detect_callbacks = []

    def __init__(self, sim, sensor_handle):
        self.sim = sim
        self.sensor_handle = sensor_handle
        self.callback = None
        self.running = False
        self._thread = None
        self.detected_objh = -1
        self.detected_objhs = set()

    @classmethod
    def on_detect(cls, func):
        cls._on_detect_callbacks.append(func)
        return func

    def _detect_objects(self):
        """Thread function to detect objects and trigger callback."""
        detected_last = False  # Track the previous detection state
        while self.running:

            detected,_,_,self.detected_objh,_ = self.sim.readProximitySensor(self.sensor_handle)
            
            
            if detected and not detected_last and self.detected_objh not in self.detected_objhs:
                self.detected_objhs.add(self.detected_objh)
                for callback in self._on_detect_callbacks:
                    callback(self, self.detected_objh)
                detected_last = True  # Mark as detected
            elif not detected:
                detected_last = False  # Reset detection state if no object is found

            time.sleep(0.1)  # Adjust polling rate for efficiency

    def start(self):
        """Start the sensor detection thread."""
        if not self.running:
            self.running = True
            self._thread = threading.Thread(target=self._detect_objects, daemon=True)
            self._thread.start()
        

    def stop(self):
        """Stop the sensor detection thread."""
        self.running = False
        if self._thread:
            self._thread.join()


conveyor = Conveyor(RemoteAPIClient().getObject('sim'),path, sample_body)

sensor = ProximitySensor(RemoteAPIClient().getObject('sim'),sim.getObject("/proximitySensor"))

ur3 = UR3(RemoteAPIClient().getObject('sim'),"/UR3")

spray = SprayGun(sim,"/PaintNozzle")
spray.clear_spray()
spray.set_spray_cone(15)

@sensor.on_detect
def _on_detect_cb(sensor,detect_objh):
    ctrl_pts = _create_spray_control_pts()

    spray_path = PathPlanner(sensor.sim,ctrl_pts)
    sensor.sim.setObjectPose(spray_path.objh,[0,0,0,0,0.7071,0,0.7071],detect_objh)
    sensor.sim.setObjectParent(spray_path.objh,detect_objh)

    spray_path.sim = ur3.sim
    ur3.stopTracking()
    ur3.createTrackingTask(spray_path,0.2)
    spray.sim = sensor.sim
    spray.on()

@ur3.on_finish_tracking
def _stop_spray(ur3):
    spray.sim = ur3.sim
    spray.off()

@conveyor.on_delete_part
def _delete_path_cb(conveyor,parentObjh):
    child_objh = sim.getObjectChild(parentObjh, 0)
        
    conveyor.sim.removeObjects([child_objh])

sensor.start()
time.sleep(0.5)


conveyor.start()
# # for i in range(100):
# #     time.sleep(1)
# #     conveyor.force_move(5)

input("Press <ENTER> to terminate")


conveyor.stop()
spray.clear_spray()
sim.stopSimulation()