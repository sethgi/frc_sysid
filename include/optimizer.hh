#pragma once

#include <vector>
#include <optional>
#include <memory>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Expression.h>

#include "sys_id_problem.hh" 
#include "sensors/apriltag.hh" 

namespace frc_sysid {


class IndexGenerator {
 public:
  int operator()() {
    return index_++;
  }
 private:
  size_t index_ = 0;
};

struct NodeInfo {
  double time;
  std::optional<AprilTagManager::Measurement> meas{std::nullopt};

  gtsam::Symbol bot_pose_symbol;
  gtsam::Symbol cam_pose_symbol;
  size_t index;

  inline static IndexGenerator index_gen;

  NodeInfo() {
    index = index_gen();
    bot_pose_symbol = gtsam::symbol_shorthand::X(index);
    cam_pose_symbol = gtsam::symbol_shorthand::C(index);
    
  }

  bool operator<(const NodeInfo& other) const {
    return time < other.time;
  }
};



class GTSAMOptimizer {
public:
  /// Construct from a reference to a system identification problem and (optionally)
  /// the number of pose intervals.
  GTSAMOptimizer(SysIdProblem &problem, size_t N_pose_intervals = 100);

  /// Build the factor graph and initial estimate.
  void setupOptimizer();

  /// Run the optimizer and return the resulting Values.
  gtsam::Values optimize();

  std::vector<Eigen::Matrix4d> getTrajectory() const;

  const std::vector<NodeInfo>& getNodes() const {
    return nodes_;
  }

  const gtsam::Values& getInitialGuess() const {return initial_estimate;}

  Eigen::Matrix4d getRobotToLimelight();

private:

  NodeInfo* getClosestNode(const double t);

  std::vector<NodeInfo> nodes_;

  // Reference to the system identification problem.
  SysIdProblem& problem_;

  // Initial guess for the robot-to-camera (limelight) transform.
  gtsam::Pose3 T_robot_limelight;
  gtsam::Symbol T_robot_limelight_sym;

  // Start and end times (from the odometry measurements).
  double start_time;
  double end_time;
  size_t N_pose_intervals;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;

  gtsam::SharedNoiseModel odom_noise;
  gtsam::SharedNoiseModel apriltag_noise;
  gtsam::SharedNoiseModel T_rob_cam_sigmas;

  std::unique_ptr<gtsam::Values> optimized_result;

};

}