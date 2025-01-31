import numpy as np
from scipy.interpolate import CubicSpline
from sensors.SensorManager import SensorManager
from config import FieldConfig, RobotConfig

class SwerveOdometryManager(SensorManager):
    def __init__(self, robot_config: RobotConfig, field_config: FieldConfig):
        super().__init__(robot_config, field_config)
        
        self.wheel_radius = self.robot_config.wheel_radius
        self.wheel_base_x = self.robot_config.wheel_base_x
        self.wheel_base_y = self.robot_config.wheel_base_y
        
        self.measurements = []
        self.splines = None
    
    def register_measurement(self, time_since_start: float, wheel_angles: np.ndarray, n_ticks: np.ndarray):
        # Convert encoder ticks to distance traveled
        wheel_distances = n_ticks * (2 * np.pi * self.wheel_radius) / 4096  # Assuming 4096 ticks per revolution
        self.measurements.append((time_since_start, wheel_angles, wheel_distances))
    
    def fit_spline(self):
        if not self.measurements:
            raise ValueError("No measurements to fit spline.")
        
        times = np.array([m[0] for m in self.measurements])
        x_positions = []
        y_positions = []
        theta_tangent_positions = []
        
        x, y, theta = 0.0, 0.0, 0.0
        for _, angles, distances in self.measurements:
            vx = np.sum(distances * np.cos(angles)) / 4
            vy = np.sum(distances * np.sin(angles)) / 4
            omega = np.sum((distances * np.sin(angles) - distances * np.cos(angles)) / self.wheel_base_x) / 4
            
            x += vx
            y += vy
            theta += omega
            x_positions.append(x)
            y_positions.append(y)
            theta_tangent_positions.append(theta)
        
        self.splines = {
            'x': CubicSpline(times, x_positions),
            'y': CubicSpline(times, y_positions),
            'theta_tangent': CubicSpline(times, theta_tangent_positions)
        }
    
    def get_pose_at_time(self, time: float):
        if self.splines is None:
            raise ValueError("Spline not fitted. Call fit_spline() first.")
        
        x = self.splines['x'](time)
        y = self.splines['y'](time)
        theta_tangent = self.splines['theta_tangent'](time)
        
        se2_matrix = np.array([
            [np.cos(theta_tangent), -np.sin(theta_tangent), x],
            [np.sin(theta_tangent),  np.cos(theta_tangent), y],
            [0,                     0,                    1]
        ])
        
        return se2_matrix
