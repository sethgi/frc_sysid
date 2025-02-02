import torch
from torch.optim import Adam
from torch.nn.functional import mse_loss
import pytorch3d.transforms
from SysIdProblem import SysIdProblem
from config import FieldConfig, RobotConfig
from optim.pose import Pose, transform_to_tensor, tensor_to_transform
import numpy as np
from typing import List

odom_cov = torch.diag_embed(torch.tensor([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
apriltag_cov = torch.diag_embed(torch.tensor([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))

class TorchOptimizer:
    def __init__(self, problem: SysIdProblem, N_pose_intervals: int = 100):
        self.problem: SysIdProblem = problem
        self.T_robot_limelight = Pose()  # Initial guess for camera-to-robot transform

        self.start_time = self.problem.odometry_manager.measurements[0][0]
        self.end_time = self.problem.odometry_manager.measurements[-1][0]
        self.N_pose_intervals = N_pose_intervals

    def setup_optimizer(self):
        # Collect times and corresponding AprilTag measurements
        pose_times = [(a[0], Pose(transform_to_tensor(a[2]))) for a in self.problem.april_tag_manager.get_measurements_robot_frame()]
        pose_times += [(t, None) for t in np.linspace(self.start_time, self.end_time, self.N_pose_intervals)]
        pose_times = sorted(pose_times)
        
        print("Poses configured.")
        
        self.apriltag_meas = [t[1] for t in pose_times]

        self.times = [t[0] for t in pose_times]
        self.poses: List[Pose] = []
        for t in self.times:
            T = torch.from_numpy(self.problem.odometry_manager.getPoseAtTime(t, se3=True))
            self.poses.append(Pose(transform_to_tensor(T)))
            
        print("Tags configured.")
        
        self.odom_measurements: List[Pose] = []
        for i in range(len(self.poses) - 1):
            self.odom_measurements.append(self.poses[i].inverse() * self.poses[i+1])
            self.odom_measurements[-1].set_fixed(True)
            
        print("initial guesses set.")
        self.T_world_tag = Pose(transform_to_tensor(self.problem.april_tag_manager.T_world_target))
        
        self.optimizable_variables = [self.T_robot_limelight.pose_tensor, self.T_world_tag.pose_tensor] + \
                                    [pose.pose_tensor for pose in self.poses]
        

    def compute_loss(self) -> torch.Tensor:
        total_loss: torch.Tensor = 0
        
        for i in range(len(self.times) - 1):
            T_world_current: Pose = self.poses[i]

            if i < len(self.times) - 1:
                T_world_next: Pose = self.poses[i+1]
                
                curr_to_next: Pose = (T_world_current.inverse() * T_world_next)
                meas: Pose = self.odom_measurements[i]
                
                diff = (meas.inverse() * curr_to_next).pose_tensor.reshape(6, 1)
                
                total_loss += diff.T @ odom_cov @ diff
        
            if self.apriltag_meas[i] is not None:
                T_rob_tag_meas = self.apriltag_meas[i]
                T_rob_tag_curr = (T_world_current * self.T_robot_limelight).inverse() * self.T_world_tag
                
                diff = (T_rob_tag_meas.inverse() * T_rob_tag_curr).pose_tensor.reshape(6, 1)
                
                total_loss += diff.T @ apriltag_cov @ diff

        return total_loss

    def optimize(self, epochs=100, lr=0.01):
        
        self.setup_optimizer()
        optimizer = Adam(self.optimizable_variables, lr=lr)

        for epoch in range(epochs):
            optimizer.zero_grad()
            loss = self.compute_loss()
            loss.backward()
            optimizer.step()

            print(f"Epoch {epoch+1}/{epochs}, Loss: {loss.item()}")


