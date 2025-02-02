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

  // Create the system identification problem.
  SysIdProblem problem(robot_config, field_config);
  
  // Process the CSV log (assumed to be in csv_directory) to register sensor data.
  problem.processLog(csv_directory);

  // Retrieve odometry trajectory (sampled as a 500x2 matrix) and AprilTag translations.
  auto odom = problem.getOdometryManager().pollSplines(500);
  Eigen::MatrixXd tags = problem.getAprilTagManager().getTranslations();

  // Assume tags is an N x 3 matrix and we want the first two columns.
  Eigen::MatrixXd apriltags = tags.block(0, 0, tags.rows(), 2);

  // Convert odometry and tag data into vectors for plotting.
  std::vector<double> odom_x, odom_y;
  for (int i = 0; i < odom.size(); i++) {
    odom_x.push_back(odom.at(i)(0, 3));
    odom_y.push_back(odom.at(i)(1, 3));
  }
  std::vector<double> tag_x, tag_y;
  for (int i = 0; i < apriltags.rows(); i++) {
    tag_x.push_back(apriltags(i, 0));
    tag_y.push_back(apriltags(i, 1));
  }

  // Create and set up the GTSAM optimizer.
  GTSAMOptimizer optimizer(problem, 500);
  optimizer.setupOptimizer();
  
  // Run the optimizer.
  auto result = optimizer.optimize();

std::vector<Eigen::Matrix4d> optimizedTrajectory = optimizer.getTrajectory();

std::vector<double> traj_x, traj_y;
for (const auto& pose : optimizedTrajectory) {
    traj_x.push_back(pose(0, 3));  // X translation component.
    traj_y.push_back(pose(1, 3));  // Y translation component.
}

// Plot the trajectories using matplotlib-cpp.
plt::figure();
plt::plot(odom_x, odom_y, {{"label", "Odometry"}});
plt::scatter(std::vector<double>{odom_x[0]}, std::vector<double>{odom_y[0]}, 50, {{"label", "Start Odometry"}, {"marker", "o"}});

plt::plot(tag_x, tag_y, {{"label", "AprilTags"}});
plt::scatter(std::vector<double>{tag_x[0]}, std::vector<double>{tag_y[0]}, 50, {{"label", "Start AprilTags"}, {"marker", "o"}});

plt::plot(traj_x, traj_y, {{"label", "Optimized Trajectory"}});
plt::scatter(std::vector<double>{traj_x[0]}, std::vector<double>{traj_y[0]}, 50, {{"label", "Start Optimized Trajectory"}, {"marker", "o"}});

plt::xlabel("X");
plt::ylabel("Y");
plt::legend();
plt::show();


std::cout << "Optimization complete." << std::endl;
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
