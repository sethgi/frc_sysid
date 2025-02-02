from config import FieldConfig, RobotConfig
from typing import Dict, Callable

from sensors.SensorManager import SensorManager
from sensors.AprilTag import AprilTagManager
from sensors.IMU import IMUManager
from sensors.Odometry import SwerveOdometryManager
from utils import load_data
import numpy as np

import matplotlib.pyplot as plt

class SysIdProblem:
    def __init__(self, robot_config: RobotConfig, field_config: FieldConfig):
        self.field_config = field_config
        self.robot_config = robot_config

        self.odometry_manager = SwerveOdometryManager(robot_config, field_config)
        self.april_tag_manager = AprilTagManager(robot_config, field_config)
        self.imu_manager = IMUManager(robot_config, field_config)

    def process_log(self, log_path):
        IMU, encoders, tags = load_data(log_path)
        
        for ts, meas in zip(*IMU):
            self.imu_manager.register_measurement(ts, meas[:3], meas[3:])

        for ts, meas in zip(*encoders):
            self.odometry_manager.register_measurement(ts, meas[:4], meas[4:])
            
        for ts, meas in zip(*tags):
            self.april_tag_manager.register_measurement(ts, None, meas)
        
        first_apriltag_time, tag, T_apriltag_robot = self.april_tag_manager._measurements[0]
        T_world_robot = self.odometry_manager.getPoseAtTime(first_apriltag_time, True)
        
        T_world_to_apriltag = T_world_robot @ np.linalg.inv(T_apriltag_robot)
        
        self.april_tag_manager.set_target_pose(T_world_to_apriltag)
        
        
        
    
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file")
    args = parser.parse_args()
    
    field_config = FieldConfig()
    robot_config = RobotConfig()
    
    problem = SysIdProblem(robot_config, field_config)
    problem.process_log(args.log_file)
    
    problem.odometry_manager.fit_spline()
    odom = problem.odometry_manager.pollSplines(500)
    apriltags = problem.april_tag_manager.get_translations()[:, :2]
    
    plt.plot(odom[:, 0], odom[:, 1], label="odometry")
    plt.plot(apriltags[:,0], apriltags[:,1], label="apriltags")
    plt.show()
    
    