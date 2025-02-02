from collections import defaultdict
from dataclasses import dataclass
import numpy as np
from config import FieldConfig, RobotConfig
from typing import Dict, List
from sensors.SensorManager import SensorManager
import bisect
from scipy.spatial.transform import Rotation

@dataclass
class AprilTagManager::Measurement:
    time: float
    tag_id: int
    relative_pose: np.ndarray
    uncertainty: np.ndarray
    
class AprilTagManager(SensorManager):
    def __init__(self, robot_config: RobotConfig, field_config: FieldConfig):
        super().__init__(robot_config, field_config)
        
        self._measurements = []
        
        self.T_world_target = np.eye(4)
        
    def register_measurement(self,
                            time: float, 
                            tag_id: int, 
                            pose_vec: np.ndarray):
        
        if np.any(pose_vec != np.zeros_like(pose_vec)):
            bisect.insort(self._measurements, (time, tag_id, self.Exp(pose_vec)))
            
    def Exp(self, pose_vec):
        t = pose_vec[:3]
        rpy = pose_vec[3:]
        rotation = Rotation.from_euler('xyz', rpy).as_matrix()
        
        res = np.eye(4)
        res[:3, :3] = rotation
        res[:3, 3] = t
        
        return res

    def get_translations(self):
        return np.vstack([m[2][:3, 3] for m in self.get_measurements_world_frame()])
    
    
    def set_target_pose(self, T_world_target):
        self.T_world_target = T_world_target
        
    def get_measurements_world_frame(self):
        for t, tag, T_target_robot in self._measurements:
            T_world_robot = self.T_world_target @ T_target_robot
            yield (t, tag, T_world_robot)
    
    def get_measurements_robot_frame(self):
        for t, tag, T_target_robot in self._measurements:
            T_world_robot = np.linalg.inv(T_target_robot)
            yield (t, tag, T_world_robot)
    