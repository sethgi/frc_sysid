import numpy as np
import json
from dataclasses import dataclass
from typing import Dict

@dataclass
class AprilTagInfo:
    tag_id: int
    tag_size: float
    field_position: np.ndarray  # 4x4 homogeneous transformation matrix

class FieldConfig:
    def __init__(self):
        self.april_tags: Dict[int, AprilTagInfo] = {}
    
    def add_april_tag(self, tag_id: int, tag_size: float, field_position: np.ndarray):
        if not isinstance(field_position, np.ndarray) or field_position.shape != (4, 4):
            raise ValueError("Field position must be a 4x4 homogeneous transformation matrix.")
        
        self.april_tags[tag_id] = AprilTagInfo(tag_id, tag_size, field_position)
    
    def get_april_tag(self, tag_id: int):
        return self.april_tags[tag_id]
    
    def load_from_json(self, json_path: str):
        with open(json_path, 'r') as f:
            data = json.load(f)
        for tag in data.get("april_tags", []):
            field_position = np.array(tag["field_position"])
            self.add_april_tag(tag["tag_id"], tag["tag_size"], field_position)

class RobotConfig:
    def __init__(self):
        self.wheel_base_x: float = None
        self.wheel_base_y: float = None
        self.wheel_radius: float = None
        self.sensors: Dict[str, np.ndarray] = {}
    
    def add_sensor_transform(self, sensor_name: str, transform: np.ndarray):
        if not isinstance(transform, np.ndarray) or transform.shape != (4, 4):
            raise ValueError("Transform must be a 4x4 homogeneous transformation matrix.")
        
        self.sensors[sensor_name] = transform
    
    def get_sensor_transform(self, sensor_name: str):
        return self.sensors.get(sensor_name, None)
    
    def load_from_json(self, json_path: str):
        with open(json_path, 'r') as f:
            data = json.load(f)
        for sensor_name, transform in data.get("sensors", {}).items():
            self.add_sensor_transform(sensor_name, np.array(transform))

        self.wheel_radius = data["wheel_radius"]
        self.wheel_base_x = data["wheel_base_x"]
        self.wheel_base_y = data["wheel_base_y"]
        
        