import gtsam
from gtsam import Pose3, BetweenFactorPose3, PriorFactorPose3, NonlinearFactorGraph, Values, noiseModel
import gtsam.noiseModel
from gtsam.symbol_shorthand import X, L, T
from SysIdProblem import SysIdProblem
from config import FieldConfig, RobotConfig
import numpy as np

class GTSAMOptimizer:
    def __init__(self, problem: SysIdProblem, N_pose_intervals: int = 100):
        self.problem: SysIdProblem = problem
        self.T_robot_limelight = Pose3()  # Initial guess for robot-to-camera transform

        self.start_time = self.problem.odometry_manager.measurements[0][0]
        self.end_time = self.problem.odometry_manager.measurements[-1][0]
        self.N_pose_intervals = N_pose_intervals

        self.graph = NonlinearFactorGraph()
        self.initial_estimate = Values()

        base_odom_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
        self.odom_noise = noiseModel.Robust.Create(noiseModel.mEstimator.Huber(1.345), base_odom_noise)
        self.apriltag_noise = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))

    def setup_optimizer(self):
        apriltag_measurements = [(a[0], Pose3(a[2])) for a in self.problem.april_tag_manager.get_measurements_robot_frame()]
        generated_times = np.linspace(self.start_time, self.end_time, self.N_pose_intervals)
        min_time_diff = (self.end_time - self.start_time) / (2 * self.N_pose_intervals)
        filtered_times = generated_times[np.insert(np.diff(generated_times) > min_time_diff, 0, True)]
        apriltag_measurements += [(t, None) for t in filtered_times]
        
        apriltag_measurements = sorted(apriltag_measurements)

        self.times = [t[0] for t in apriltag_measurements]
        self.apriltag_measurements = [t[1] for t in apriltag_measurements]

        self.poses = []

        ## Initial Estimate
        
        # Poses
        for idx, t in enumerate(self.times):
            T_world_rob_i = Pose3(self.problem.odometry_manager.getPoseAtTime(t, se3=True))
            self.poses.append(T_world_rob_i)
            self.initial_estimate.insert(X(idx), T_world_rob_i)

        # Extrinsics
        self.initial_estimate.insert(T(0), self.T_robot_limelight)


        ## Factors
        
        # Initial factor on pose 0
        self.graph.add(PriorFactorPose3(X(0), self.poses[0], noiseModel.Isotropic.Sigma(6,0)))

        # Odometry factors
        for i in range(len(self.times) - 1):
            odom_meas = self.poses[i].between(self.poses[i+1])
            self.graph.add(BetweenFactorPose3(X(i), X(i+1), odom_meas, self.odom_noise))

        # Apriltag measurements
        for i, t in enumerate(self.times):
            apriltag_meas = self.apriltag_measurements[i]
            if apriltag_meas:
                tag_pose = Pose3(apriltag_meas)
                expression = gtsam.ExpressionPose3(X(i)).compose(gtsam.ExpressionPose3(T(0))).inverse().compose(gtsam.ExpressionPose3(L(0)))
                factor = gtsam.NonlinearEqualityPose3(expression, tag_pose)
                self.graph.add(factor)

    def optimize(self):
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate, params)
        result = optimizer.optimize()
        
        breakpoint()

        print(f"Final error: {self.graph.error(result)}")
        return result
