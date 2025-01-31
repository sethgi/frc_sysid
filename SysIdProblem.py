from config import FieldConfig, RobotConfig
from typing import Dict, Callable

from sensors.SensorManager import SensorManager
from sensors.AprilTag import AprilTagManager
from sensors.IMU import IMUManager
from sensors.Odometry import SwerveOdometryManager
from DataLog import load_data

class SysIdProblem:
    def __init__(self, field_config: FieldConfig, robot_config: RobotConfig):
        self.field_config = field_config
        self.robot_config = robot_config

        self.odometry_manager = SwerveOdometryManager(field_config, robot_config)
        self.april_tag_manager = AprilTagManager(field_config, robot_config)
        self.imu_manager = IMUManager(field_config, robot_config)

    def process_log(self, log_data):
        IMU, encoders, tags = load_data(log_data)
        
        for ts, meas in zip(*IMU):
            self.imu_manager.register_measurement(ts, meas[:3], meas[3:])

        for ts, meas in zip(*encoders):
            self.odometry_manager.register_measurement(ts, encoders[:4], encoders[4:])
            
        for ts, meas in zip(*tags):
            self.april_tag_manager.register_measurement(ts, None, meas)
        
        