from dataclasses import dataclass
import numpy as np
from sensors.SensorManager import SensorManager

@dataclass
class IMUMeasurement:
    time: float
    acceleration: np.ndarray  # 3D acceleration vector
    gyroscope: np.ndarray     # 3D angular velocity vector

class IMUManager(SensorManager):
    def __init__(self, *args):
        super().__init__(*args)
        self.measurements = []
    
    def register_measurement(self, time: float, acceleration: np.ndarray, gyroscope: np.ndarray):
        if not isinstance(acceleration, np.ndarray) or acceleration.shape != (3,):
            raise ValueError("Acceleration must be a 3-element vector.")
        
        if not isinstance(gyroscope, np.ndarray) or gyroscope.shape != (3,):
            raise ValueError("Gyroscope data must be a 3-element vector.")
        
        self.measurements.append(IMUMeasurement(time, acceleration, gyroscope))
    
    def get_latest_measurement(self):
        if not self.measurements:
            raise ValueError("No measurements recorded.")
        return self.measurements[-1]
    
    def get_measurements_in_time_range(self, start_time: float, end_time: float):
        return [m for m in self.measurements if start_time <= m.time <= end_time]
