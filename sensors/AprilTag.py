from collections import defaultdict
from dataclasses import dataclass
import numpy as np
from config import FieldConfig, RobotConfig
from typing import Dict, List
from sensors.SensorManager import SensorManager

@dataclass
class AprilTagMeasurement:
    time: float
    tag_id: int
    relative_pose: np.ndarray
    uncertainty: np.ndarray
    
class AprilTagManager(SensorManager):
    def __init__(self, robot_config: RobotConfig, field_config: FieldConfig):
        super().__init__(robot_config, field_config)
        
        self.measurements: Dict[int, List[AprilTagMeasurement]] = \
            defaultdict(lambda: []) # tag_id : measurement
        
    # def register_measurement(self,
    #                          time: float, 
    #                          tag_id: int, 
    #                          relative_pose: np.ndarray,
    #                          uncertainty: np.ndarray):
        
    #     if not isinstance(relative_pose, np.ndarray) or relative_pose.shape != (4,4):
    #         raise ValueError("The pose must be a 4x4 homogemous transformation matrix.")
        
    #     if not isinstance(uncertainty, np.ndarray) or uncertainty.shape != (6,):
    #         raise ValueError("The uncertainty must be a 6-vector in the tangent space.")
        
    #     self.measurements[tag_id].append(AprilTagMeasurement(time, tag_id, relative_pose, uncertainty))

    def register_measurement(self,
                            time: float, 
                            tag_id: int, 
                            pose_vec: np.ndarray):
    
        pass