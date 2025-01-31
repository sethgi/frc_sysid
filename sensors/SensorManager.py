from abc import ABC, abstractmethod
from config import RobotConfig, FieldConfig

class SensorManager(ABC):
    def __init__(self, robot_config: RobotConfig, field_config: FieldConfig):
        self.robot_config: RobotConfig = robot_config
        self.field_config: FieldConfig = field_config
        
    @abstractmethod
    def register_measurement(self):
        return NotImplemented
    
    @abstractmethod
    def build_factors(self):
        return NotImplemented