#include "optimizer.hh"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <ranges>

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
  Vector sigmas = (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished();
  auto base_odom_noise = noiseModel::Diagonal::Sigmas(sigmas);

  Vector april_sigmas = (Vector(3) << 0.25,0.25,0.25).finished();
  auto base_apriltag_noise = noiseModel::Diagonal::Sigmas(april_sigmas);

  Vector t_rob_cam_sigmas = (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.01).finished();
  T_rob_cam_sigmas = noiseModel::Diagonal::Sigmas(sigmas);


  odom_noise = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), base_odom_noise);
  apriltag_noise = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), base_apriltag_noise);

  T_robot_limelight_sym = T(0);

}


NodeInfo* GTSAMOptimizer::getClosestNode(const double target_time)
{
  if (nodes_.empty()) {
    std::cerr << "No nodes present" << std::endl;
    return nullptr;
  }
  // Binary search for the first node with time >= target_time
  auto it = std::lower_bound(nodes_.begin(), nodes_.end(), target_time, 
                             [](const NodeInfo& node, double t) {
                               return node.time < t;
                             });

  // Handle edge cases
  if (it == nodes_.begin()) return &(*it);                   // target_time is before all nodes
  if (it == nodes_.end()) return &nodes_.back();              // target_time is after all nodes

  // Compare the current and previous nodes to find the closest
  auto prev_it = std::prev(it);
  if (std::abs(it->time - target_time) < std::abs(prev_it->time - target_time)) {
    return &(*it);
  } else {
    return &(*prev_it);
  }
}

//---------------------------------------------------------------------
// Build the factor graph and initial estimate.
void GTSAMOptimizer::setupOptimizer() {
  const auto &odomMeas = problem_.getOdometryManager().getMeasurements();
  start_time = odomMeas.front().time;
  end_time = odomMeas.back().time;

  // Optimization Variables: Location of each target. We optimize just as a point for now.
  std::map<int, gtsam::Symbol> t_world_tag_syms;

  // Calibration: transform from robot to camera.
  uint calib_symbols{0};
  T_robot_limelight_sym = T(calib_symbols++);

  gtsam::Pose3 T_robot_cam(problem_.getRobotConfig().T_robot_camera);


  nodes_.clear();

  for(const auto& april_tag_meas : problem_.getAprilTagManager().getMeasurementsCameraFrame())
  {
    NodeInfo new_node;
    new_node.time = april_tag_meas.time;
    new_node.meas = std::make_optional(april_tag_meas);
    nodes_.push_back(new_node);
  }

  std::sort(nodes_.begin(), nodes_.end());

  const auto timesteps = linspace(start_time, end_time, N_pose_intervals);

  for(const auto t : timesteps) {
    const NodeInfo* closest_existing_node = getClosestNode(t);
    if(std::fabs(closest_existing_node->time - t) > (end_time - start_time) * N_pose_intervals / 2)
    {
      NodeInfo new_node;
      new_node.time = t;
      nodes_.push_back(new_node);
    }
  }

  std::sort(nodes_.begin(), nodes_.end());

  std::cout << "Number of Nodes: " << nodes_.size() << std::endl;

  // Initial Guesses and Factors
  initial_estimate.clear();
  graph = gtsam::NonlinearFactorGraph();

  initial_estimate.insert(T_robot_limelight_sym, T_robot_cam);
  auto prior_noise_T = gtsam::noiseModel::Diagonal::Sigmas((
    gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1, 1, 1).finished());

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(T_robot_limelight_sym, 
                                             T_robot_cam,
                                             prior_noise_T));

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    nodes_.front().bot_pose_symbol,
    Pose3(),
    prior_noise_T));

  for(int node_idx = 0; node_idx < nodes_.size(); ++node_idx) {
    const auto& node = nodes_.at(node_idx);

    const gtsam::Pose3 bot_pose(problem_.getOdometryManager().getPoseAtTime(node.time));
    initial_estimate.insert(node.bot_pose_symbol, 
                            bot_pose);

    // Initial guess on the camera nodes
    const auto cam_pose = bot_pose * T_robot_cam;
    initial_estimate.insert(node.cam_pose_symbol,
                            bot_pose * T_robot_cam);

    Eigen::VectorXd flat_sigmas(6);

    flat_sigmas << 1e6, 1e6, 1e-3, 1e6, 1e6, 1e6; // Constrain z, roll, pitch tightly, leave x, y, yaw unconstrained
    auto flat_noise = gtsam::noiseModel::Diagonal::Sigmas(flat_sigmas);


    // Odometry factors
    if(node_idx != nodes_.size() - 1) {
      const gtsam::Pose3 next_pose(
        problem_.getOdometryManager().getPoseAtTime(nodes_.at(node_idx+1).time));

      gtsam::Pose3 between_measurement = bot_pose.inverse() * next_pose;

      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        nodes_.at(node_idx).bot_pose_symbol,       // Current pose symbol
        nodes_.at(node_idx + 1).bot_pose_symbol,   // Next pose symbol
        between_measurement,                       // Relative transformation
        odom_noise                                 // Noise model
      ));
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(nodes_.at(node_idx).bot_pose_symbol, 
                                                 gtsam::Pose3(),
                                                 flat_noise));

    }

    // Factor to connect bot pose to camera pose
    gtsam::Expression<gtsam::Pose3> bot_pose_expr(node.bot_pose_symbol);
    gtsam::Expression<gtsam::Pose3> T_robot_cam_expr(T_robot_limelight_sym);
    gtsam::Expression<gtsam::Pose3> cam_pose_expr(node.cam_pose_symbol);

    // Expression representing T_world_rob * T_rob_cam
    gtsam::Expression<gtsam::Pose3> composed_expr = gtsam::compose(bot_pose_expr, T_robot_cam_expr);

    // Add the equality constraint: T_world_rob * T_rob_cam = T_world_cam
    graph.add(gtsam::ExpressionFactor<gtsam::Pose3>(
      T_rob_cam_sigmas,                      // Noise model
      gtsam::Pose3(),                        // The expected value is identity (no error)
      gtsam::between(composed_expr, cam_pose_expr) // Error between composed pose and cam pose
    ));

    // Deal with apriltag measurements
    if(node.meas.has_value())
    {
      const auto& tag_id = node.meas->tag_id;
      if(!t_world_tag_syms.contains(tag_id)) {
        const auto tag_pos = cam_pose.transformFrom(gtsam::Point3(
            node.meas->bearing_vec * node.meas->range));
            
        t_world_tag_syms.emplace(tag_id, L(t_world_tag_syms.size()));
        initial_estimate.insert(t_world_tag_syms.at(tag_id), tag_pos);
      }
      auto bearing = gtsam::Unit3(node.meas->bearing_vec);
      auto range = node.meas->range;
      graph.add(gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>(
              node.cam_pose_symbol, t_world_tag_syms.at(tag_id), bearing, range, apriltag_noise));
    }
  }
  graph.print("Graph: \n");

}

//---------------------------------------------------------------------
// Optimize the graph using Levenberg–Marquardt.
Values GTSAMOptimizer::optimize() {
  LevenbergMarquardtParams params;
  params.verbosity = LevenbergMarquardtParams::Verbosity::ERROR;
  params.maxIterations = 1000;

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

  for (const auto& node : nodes_) {
    const auto sym = node.bot_pose_symbol;
    if (optimized_result->exists(sym)) {
      Pose3 pose = optimized_result->at<Pose3>(sym);
      trajectory.push_back(pose.matrix());
    }
  }
  return trajectory;
}

Eigen::Matrix4d GTSAMOptimizer::getRobotToLimelight()
{
  return optimized_result->at<Pose3>(T_robot_limelight_sym).matrix();
}

}