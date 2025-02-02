import numpy as np
from scipy.interpolate import CubicSpline
from sensors.SensorManager import SensorManager
from config import FieldConfig, RobotConfig
import matplotlib.pyplot as plt
import bisect

class SwerveOdometryManager(SensorManager):
    def __init__(self, robot_config: RobotConfig, field_config: FieldConfig):
        super().__init__(robot_config, field_config)
        
        self.wheel_radius = self.robot_config.wheel_radius
        self.wheel_base_x = self.robot_config.wheel_base_x
        self.wheel_base_y = self.robot_config.wheel_base_y
        
        self.measurements = []
        self.splines = None
        self.prev_dist = None
    
    def register_measurement(self, time_since_start: float, wheel_angles: np.ndarray, wheel_dist: np.ndarray):
        if self.prev_dist is None:
            self.prev_dist = wheel_dist
        
        # Compute delta distances from accumulated encoder values
        wheel_distances = wheel_dist - self.prev_dist
        self.prev_dist = wheel_dist
        
        bisect.insort(self.measurements, (time_since_start, wheel_angles, wheel_distances))
        self.splines = None
    
    def fit_spline(self):
        if not self.measurements:
            raise ValueError("No measurements to fit spline.")
        
        times = np.array([m[0] for m in self.measurements])
        x_positions, y_positions, theta_positions = [], [], []
        
        x, y, theta = 0.0, 0.0, 0.0
        
        for _, angles, distances in self.measurements:
            vx = np.mean(distances * np.cos(angles))
            vy = np.mean(distances * np.sin(angles))
            omega = np.mean((distances * np.sin(angles) - distances * np.cos(angles)) / self.wheel_base_x)
            
            x += vx
            y += vy
            theta += omega
            
            x_positions.append(x)
            y_positions.append(y)
            theta_positions.append(theta)
        
        self.splines = {
            'x': CubicSpline(times, x_positions),
            'y': CubicSpline(times, y_positions),
            'theta': CubicSpline(times, theta_positions)
        }
    
    def getPoseAtTime(self, time: float, se3: bool = False, tensor: bool = False):
        if self.splines is None:
            self.fit_spline()
        
        x = self.splines['x'](time)
        y = self.splines['y'](time)
        theta = self.splines['theta'](time)
        
        
        if se3:
            pose = np.array([
                [np.cos(theta), -np.sin(theta), 0, x],
                [np.sin(theta),  np.cos(theta), 0, y],
                [0,             0,             1, 0],
                [0,0,0,1]
            ]) 
        else:
            pose = np.array([
                [np.cos(theta), -np.sin(theta), x],
                [np.sin(theta),  np.cos(theta), y],
                [0,             0,             1]
            ])
        
        return pose

    def pollSplines(self, n_pts):
        if self.splines is None:
            self.fit_spline()
        
        times = np.linspace(self.measurements[0][0], self.measurements[-1][0], n_pts)
        poses = [self.getPoseAtTime(t) for t in times]
        x_vals = np.array([pose[0, 2] for pose in poses])
        y_vals = np.array([pose[1, 2] for pose in poses])
        
        return np.vstack((x_vals, y_vals)).T
