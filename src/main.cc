#include "config.hh"         // Defines RobotConfig
#include "sys_id_problem.hh"        // Defines SysIdProblem and its accessors.
#include "optimizer.hh"      // Defines GTSAMOptimizer.

#include "thirdparty/matplotlibcpp.h"       // Header-only matplotlib-cpp library

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace plt = matplotlibcpp;

using namespace frc_sysid;


void run(const std::string &csv_directory) {
  plt::backend("TkAgg");

  // Create configuration objects.
  FieldConfig field_config;  // Populate as needed.
  RobotConfig robot_config;  // Populate as needed (e.g., set wheel_radius, wheel_base_x, etc.).

  Eigen::Matrix4d T_rob_cam = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R_world_robot = Eigen::AngleAxisd(-deg2rad(30.), Eigen::Vector3d::UnitY()).toRotationMatrix();
  T_rob_cam.block<3,3>(0,0) = R_world_robot;
  T_rob_cam(0,3) = 0.381;
  T_rob_cam(2,3) = 0.305;
  robot_config.T_robot_camera = T_rob_cam;  

  // Create the system identification problem.
  SysIdProblem problem(robot_config, field_config);
  
  // Process the CSV log (assumed to be in csv_directory) to register sensor data.
  problem.processLog(csv_directory);

  // Retrieve odometry trajectory (sampled as a 500x2 matrix) and AprilTag translations.
  auto odom = problem.getOdometryManager().pollSplines(500);

  // Create and set up the GTSAM optimizer.
  GTSAMOptimizer optimizer(problem, 20000);
  optimizer.setupOptimizer();
  
  // Run the optimizer.
  auto initial = optimizer.getInitialGuess();
  
  auto result = optimizer.optimize();

  auto nodes = optimizer.getNodes();

  std::ofstream tum_file("optimized_trajectory_tum.txt");
  if (!tum_file.is_open()) {
      std::cerr << "Failed to open output file for TUM trajectory." << std::endl;
      return;
  }

  // Log poses in TUM format
  for (const auto& node : nodes) {
      if (result.exists(node.bot_pose_symbol)) {
          gtsam::Pose3 pose = result.at<gtsam::Pose3>(node.bot_pose_symbol);
          
          double timestamp = node.time;
          gtsam::Point3 t = pose.translation();
          gtsam::Quaternion q = pose.rotation().toQuaternion();  // Converts to (w, x, y, z)
          
          // Write to file in TUM format: timestamp tx ty tz qx qy qz qw
          tum_file << std::fixed << timestamp << " "
                  << t.x() << " " << t.y() << " " << t.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      }
  }

  tum_file.close();
  std::cout << "Optimized trajectory saved in TUM format." << std::endl;

  std::vector<double> odom_x, odom_y;
  std::vector<double> traj_x, traj_y;
  std::vector<double> meas_x, meas_y;
  std::vector<std::pair<std::vector<double>, std::vector<double>>> measurement_lines;

  for (const auto& node : nodes) {
    // Extract Odometry positions (assuming odometry is in bot_pose_symbol).
    const auto& odom_pose = problem.getOdometryManager().getPoseAtTime(node.time);
    odom_x.push_back(odom_pose(0, 3));
    odom_y.push_back(odom_pose(1, 3));

    // Extract Optimized trajectory positions.
    gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(node.bot_pose_symbol);
    traj_x.push_back(optimized_pose.x());
    traj_y.push_back(optimized_pose.y());

    // Extract and plot AprilTag measurements.
    if (node.meas.has_value()) {
      const auto& measurement = node.meas.value();

      gtsam::Point3 local_measurement = measurement.range * measurement.bearing_vec;

      // Transform measurement from robot frame to world frame.
      gtsam::Point3 world_measurement = optimized_pose.transformFrom(local_measurement);

      meas_x.push_back(world_measurement.x());
      meas_y.push_back(world_measurement.y());

      // Store range/bearing vectors for plotting.
      measurement_lines.push_back({
        {optimized_pose.x(), world_measurement.x()},
        {optimized_pose.y(), world_measurement.y()}
      });
    }
  }

  // Plot Odometry trajectory.
  plt::plot(odom_x, odom_y, {{"label", "Odometry"}});
  plt::scatter(std::vector<double>{odom_x[0]}, std::vector<double>{odom_y[0]}, 50, {{"label", "Start Odometry"}, {"marker", "o"}});

  // Plot Optimized trajectory.
  plt::plot(traj_x, traj_y, {{"label", "Optimized Trajectory"}});
  plt::scatter(std::vector<double>{traj_x[0]}, std::vector<double>{traj_y[0]}, 50, {{"label", "Start Optimized Trajectory"}, {"marker", "o"}});

  // Plot AprilTag measurements.
  plt::scatter(meas_x, meas_y, 30, {{"label", "AprilTag Measurements"}, {"marker", "x"}});
  if (!meas_x.empty() && !meas_y.empty()) {
    plt::scatter(std::vector<double>{meas_x[0]}, std::vector<double>{meas_y[0]}, 50, {{"marker", "o"}});
  }

  // Draw measurement vectors from robot pose to measurement location.
  for (const auto& line : measurement_lines) {
    plt::plot(line.first, line.second, {{"color", "gray"}, {"linestyle", "--"}});
  }

  // Final plot setup.
  plt::xlabel("X");
  plt::ylabel("Y");
  plt::legend();
  plt::show();


  std::cout << "Optimization complete." << std::endl;

  Eigen::Matrix4d T_robot_cam = optimizer.getRobotToLimelight();

  printPose(robot_config.T_robot_camera, "Initial T_robot_cam");
  printPose(T_robot_cam, "Optimized T_robot_cam");

}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <csv_directory>" << std::endl;
    return 1;
  }

  std::string csv_directory = argv[1];

  run(csv_directory);

  return 0;
}
