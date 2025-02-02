#include "optimizer.hh"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

#include <iostream>
#include <algorithm>
#include <cmath>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;

#define CHECK_NAN(matrix) \
  if ((matrix).array().isNaN().any()) { \
    std::cout << #matrix " contains NaN elements." << std::endl; \
  }

namespace frc_sysid {

std::vector<double> linspace(double start, double end, size_t num) {
  std::vector<double> result;
  if(num == 0) return result;
  if(num == 1) {
    result.push_back(start);
    return result;
  }
  double step = (end - start) / (num - 1);
  for (size_t i = 0; i < num; i++) {
    result.push_back(start + i * step);
  }
  return result;
}


GTSAMOptimizer::GTSAMOptimizer(SysIdProblem &problem, size_t N_pose_intervals)
  : problem_(problem), N_pose_intervals(N_pose_intervals), T_robot_limelight()
{
  // Here we assume that problem_.getOdometryManager().getMeasurements() returns a
  // vector of odometry measurements sorted by time, and that each measurement has a "time" field.
  const auto &odomMeas = problem_.getOdometryManager().getMeasurements();
  if (odomMeas.empty()) {
    throw std::runtime_error("No odometry measurements available.");
  }
  start_time = odomMeas.front().time;
  end_time = odomMeas.back().time;

  // Create noise models.
  // Here we set all six sigmas to 0.1.
  Vector sigmas = (Vector(6) << 0.01, 0.01, 0.1, 0.1, 0.1, 0.01).finished();
  auto base_odom_noise = noiseModel::Diagonal::Sigmas(sigmas);

  Vector april_sigmas = (Vector(6) << 1,1,1,1,1,1).finished();
  auto base_apriltag_noise = noiseModel::Diagonal::Sigmas(april_sigmas);


  odom_noise = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), base_odom_noise);
  apriltag_noise = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), base_apriltag_noise);

  sym_robot_to_limelight = T(0);

}

//---------------------------------------------------------------------
// Build the factor graph and initial estimate.
void GTSAMOptimizer::setupOptimizer() {
  // Build a vector of AprilTag measurement pairs:
  // For each measurement from the AprilTag manager (in robot frame),
  // store the measurement time and a Pose3 constructed from its relative pose.
  std::vector<std::pair<double, std::optional<Pose3>>> times_and_apriltags;
  for (const auto &m : problem_.getAprilTagManager().getMeasurementsWorldFrame()) {
    times_and_apriltags.emplace_back(m.time, Pose3(m.transform));
  }

  // Generate additional times from start_time to end_time.
  std::vector<double> generated_times = linspace(start_time, end_time, N_pose_intervals);
  double min_time_diff = (end_time - start_time) / (2.0 * N_pose_intervals);
  

  std::vector<double> filtered_times = generated_times; // For simplicity.

  // Append pairs (t, nullopt) for each generated time.
  for (double t : filtered_times) {
    times_and_apriltags.emplace_back(t, std::nullopt);
  }

  // Sort the combined vector by time.
  std::sort(times_and_apriltags.begin(), times_and_apriltags.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });


  const Pose3 T_world_bot_initial(problem_.getAprilTagManager().getFirstBotPose());

  poses.clear();
  // Initial Estimate: For each time, get the robot pose from odometry.
  for (size_t idx = 0; idx < times_and_apriltags.size(); idx++) {
    const Pose3 T_initial_bot_i(problem_.getOdometryManager().getPoseAtTime(times_and_apriltags.at(idx).first));
    const Pose3 T_world_bot_i = T_world_bot_initial * T_initial_bot_i;
    poses.push_back(T_world_bot_i);
    
    CHECK_NAN(T_world_bot_i.matrix())

    initial_estimate.insert(X(idx), T_world_bot_i);
  }


  initial_estimate.insert(sym_robot_to_limelight, T_robot_limelight);
  CHECK_NAN(T_robot_limelight.matrix())

  if(poses.size() == 0) {
    throw std::logic_error("There are no poses :(");
  }

  // Add factors:
  // 1. Prior factor on pose 0.
  graph.add(std::make_shared<PriorFactor<Pose3>>(X(0), poses[0],
                              noiseModel::Isotropic::Sigma(6, 0.3)));

  // 2. Odometry (Between) factors.
  for (size_t i = 0; i < times_and_apriltags.size() - 1; i++) {
    Pose3 odom_meas = poses[i].between(poses[i+1]);
    CHECK_NAN(odom_meas.matrix())
    graph.add(std::make_shared<BetweenFactor<Pose3>>(X(i), X(i+1), odom_meas, odom_noise));
  }

  

  // 3. AprilTag measurement factors.  
  for (size_t i = 0; i < times_and_apriltags.size(); i++) {
    if (times_and_apriltags.at(i).second.has_value()) {
      Pose3 T_world_to_limelight = times_and_apriltags.at(i).second.value();

      Expression<Pose3> expr_world_to_rob(X(i));
      Expression<Pose3> expr_rob_to_limelight(sym_robot_to_limelight);
      Expression<Pose3> expr_world_to_limelight = gtsam::compose(
        expr_world_to_rob, expr_rob_to_limelight);

      auto factor = std::make_shared<ExpressionFactor<Pose3>>(apriltag_noise, T_world_to_limelight, expr_world_to_limelight);
      graph.add(factor);
    }
  }
}

//---------------------------------------------------------------------
// Optimize the graph using Levenberg–Marquardt.
Values GTSAMOptimizer::optimize() {
  LevenbergMarquardtParams params;
  params.verbosity = LevenbergMarquardtParams::Verbosity::ERROR;

  LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);

  std::cout << "Initial error:" << graph.error(initial_estimate) << std::endl;

  Values result = optimizer.optimize();

  std::cout << "Final error: " << graph.error(result) << std::endl;

  optimized_result = std::make_unique<gtsam::Values>(result);

  return result;
}

std::vector<Eigen::Matrix4d> GTSAMOptimizer::getTrajectory() const {
  if(!optimized_result) {
    throw std::logic_error("Can't getTrajectory() without having optimized yet.");

  }
  std::vector<Eigen::Matrix4d> trajectory;

  for (size_t i = 0; i < poses.size(); i++) {
    Symbol sym = X(i);
    if (optimized_result->exists(sym)) {
      Pose3 pose = optimized_result->at<Pose3>(sym);
      trajectory.push_back(pose.matrix());
    }
  }
  return trajectory;
}

}