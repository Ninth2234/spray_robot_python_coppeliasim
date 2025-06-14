import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import threading

class PathPlanner:
    def __init__(self,sim,ctrl_pts,smoothness=0.8,create_new_path=True,exist_path_handle=None):
        """ create path

        Args:
            sim (_type_): sim object
            ctrl_pts (float []): [x0,y0,z0,qx0,qy0,qz0,qw0,x1,y1,z1,....]
            smoothness (float): 0.0 (rough), 1.0 (smooth)
        """
        self.sim = sim
        self.ctrl_pts = ctrl_pts

        if create_new_path:
            self.objh = self.sim.createPath(ctrl_pts,8,100,smoothness)
        else:
            self.objh = exist_path_handle

        self.path = sim.unpackDoubleTable(sim.getBufferProperty(self.objh, 'customData.PATH'))
        self.pathLengths,self.totalLength = sim.getPathLengths(self.path,7)

    def get_interpolate_pose(self,ds:float):
        """get [x,y,z,qx,qy,qz,qw] from path at given length (ds)

        Args:
            ds (float): length

        Returns:
            list float: _description_
        """
        # if ds > self.totalLength:
        #     ds = self.totalLength
        pose_to_path = self.sim.getPathInterpolatedConfig(self.path, self.pathLengths, ds)
        pose = self.sim.multiplyPoses(self.sim.getObjectPose(self.objh),pose_to_path)
        return pose

    def get_length(self):
        return self.totalLength

    


class UR3:
    BASE = 0
    WORLD = 1

    MAX_VEL = 0.2
    MAX_ACCEL = 1
    MAX_JERK = 1

    MAX_JOINT_VEL = 100*math.pi/180
    MAX_JOINT_ACCEL = 100*math.pi / 180
    MAX_JOINT_JERK = 100*math.pi / 180

    _start_tracking_callbacks = []
    _finish_tracking_callbacks = []

    def __init__(self, sim, name):
        self.sim = sim
        self.name = name

        self.baseh = sim.getObject(name)
        self.scripth = sim.getObject(name+"/Script")
        self.jointh = [sim.getObject(name+"/joint",{'index':i}) for i in range(6)]
        self.effh = sim.getObject(name+"/manipSphere")
        self.tiph = sim.getObject(name+"/tip")


        self._tracking_thread = None
        self._tracking_stop_event = threading.Event()  # Event for stopping the thread


    def reset_target(self):
        self.sim.setObjectPose(self.effh, [0,0,0,0,0,0,1], self.tiph)

    def get_pose(self, relative_to=BASE):
        """return pose (position,quaternion) [x y z qx qy qz qw]

        Args:
            relative_to : UR3.BASE or UR3.WORLD. Defaults to BASE.

        Returns:
            list: [x y z qx qy qz qw]
        """
        if relative_to == UR3.BASE:
            return self.sim.getObjectPose(self.tiph, self.baseh)
        if relative_to == UR3.WORLD:
            return self.sim.getObjectPose(self.tiph)
        
    def get_joint(self):

        return [self.sim.getJointPosition(joint) for joint in self.jointh]
    
    def move_pose(self, pose, vel=None, accel=None, jerk=None, ref_to_world=False):
        """Move robot with Cartesian coordinates using sim.moveToPose.

        Args:
            pose (list[7]): Target pose [x, y, z, qx, qy, qz, qw] in base frame.
            vel (list[4], optional): Max velocity [vx, vy, vz, vw]. Defaults to None.
            accel (list[4], optional): Max acceleration [ax, ay, az, aw]. Defaults to None.
            jerk (list[4], optional): Max jerk [jx, jy, jz, jw]. Defaults to None.
            ref_to_world (boolean): If true, pose is referenced to world
        """
        
        if ref_to_world:
            pose_to_world = pose
        else:
            pose_to_world = self.sim.multiplyPoses(self.sim.getObjectPose(self.baseh),pose)

        if vel is None:
            vel = [UR3.MAX_VEL for _ in range(4)]
        if accel is None:
            accel = [UR3.MAX_ACCEL for _ in range(4)]
        if jerk is None:
            jerk = [UR3.MAX_JERK for _ in range(4)]
        
        param = {
            "targetPose": pose_to_world,
            "ik": {"tip":self.tiph,"base":self.baseh,"target":self.effh},
            'maxVel': vel,
            'maxAccel': accel,
            'maxJerk': jerk,            
        }
        self.sim.moveToPose(param)


    def move_joint(self, q, vel=None,accel=None,jerk=None):

        if vel is None:
            vel = [UR3.MAX_JOINT_VEL for _ in range(len(self.jointh))]
        if accel is None:
            accel = [UR3.MAX_JOINT_ACCEL for _ in range(len(self.jointh))]
        if jerk is None:
            jerk = [UR3.MAX_JOINT_JERK for _ in range(len(self.jointh))]
        
        param = {
            'joints':self.jointh,
            'targetPos':q,
            'maxVel':vel,
            'maxAccel':accel,
            'maxJerk':jerk
        }
        
        self._set_joint_mode()
        self.sim.moveToConfig(param)
        self._set_ik_mode()

    def _set_joint_mode(self):
        sim.callScriptFunction("set_joint_mode",self.scripth)

    def _set_ik_mode(self):
        sim.callScriptFunction("set_ik_mode",self.scripth)

    @classmethod
    def on_start_tracking(cls,func):
        cls._start_tracking_callbacks.append(func)
        return func
    
    @classmethod
    def on_finish_tracking(cls,func):
        cls._finish_tracking_callbacks.append(func)
        return func

    def tracking(self,path:PathPlanner,vel:float):
        """following the input path (blocking).

        Args:
            path (PathPlanner): given path
            vel (float): tracking velocity
        """
        t0 = self.sim.getSimulationTime()

        while (ds:=vel*(self.sim.getSimulationTime()-t0)) < path.get_length():
            target_pose = path.get_interpolate_pose(ds)
            self.sim.setObjectPose(self.effh,target_pose)

        target_pose = self.sim.getPathInterpolatedConfig(path.path, path.pathLengths, ds)    
        self.sim.setObjectPose(self.effh,target_pose)

    def createTrackingTask(self, path: PathPlanner, vel: float):
        """Create a non-blocking task to follow a given path.

        Args:
            path (PathPlanner): The path to follow.
            vel (float): The desired tracking velocity.
        """
        self._tracking_stop_event.clear()  # Reset stop event

        def tracking_task():
            t0 = self.sim.getSimulationTime()
            while not self._tracking_stop_event.is_set():
                ds = vel * (self.sim.getSimulationTime() - t0)
                if ds >= path.get_length():
                    break  # Stop when reaching the end of the path

                target_pose = path.get_interpolate_pose(ds)
                self.sim.setObjectPose(self.effh, target_pose)
                time.sleep(0.05)  # Avoid busy waiting

            # Ensure the final pose is set
            target_pose = path.get_interpolate_pose(ds)
            self.sim.setObjectPose(self.effh, target_pose)

            for callback in self._finish_tracking_callbacks:
                callback(self)

        for callback in self._start_tracking_callbacks:
            callback(self)

        # Start the tracking thread
        self._tracking_thread = threading.Thread(target=tracking_task, daemon=True)
        self._tracking_thread.start()

    def stopTracking(self):
        """Stop the tracking task if running."""
        if self._tracking_thread and self._tracking_thread.is_alive():
            self._tracking_stop_event.set()  # Signal the thread to stop
            self._tracking_thread.join()  # Wait for thread to finish
            self._tracking_thread = None

    






if __name__ == "__main__":
    

    from numpy import linspace
    import numpy as np
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

