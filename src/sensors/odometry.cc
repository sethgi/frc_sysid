#include "sensors/odometry.hh"

namespace frc_sysid {

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Constructor
SwerveOdometryManager::SwerveOdometryManager(const RobotConfig &robot_config, const FieldConfig &field_config)
    : SensorManager(robot_config, field_config),
      wheel_radius(robot_config.wheel_radius),
      wheel_base_x(robot_config.wheel_base_x),
      wheel_base_y(robot_config.wheel_base_y),
      prev_dist_initialized(false),
      splinesInitialized(false)
{}

// Register a new measurement.
void SwerveOdometryManager::registerMeasurement(double time_since_start,
                                                 const Eigen::Vector4d &wheel_angles,
                                                 const Eigen::Vector4d &wheel_dist)
{
    if (!prev_dist_initialized) {
        prev_dist = wheel_dist;
        prev_dist_initialized = true;
    }
    // Compute delta distances from accumulated encoder values.
    Eigen::Vector4d wheel_distances = wheel_dist - prev_dist;
    prev_dist = wheel_dist;
    
    Measurement m { time_since_start, wheel_angles, wheel_distances };
    auto it = std::lower_bound(measurements.begin(), measurements.end(), m,
                               [](const Measurement &a, const Measurement &b) {
                                   return a.time < b.time;
                               });
    measurements.insert(it, m);
    
    // Invalidate any previously computed spline.
    splinesInitialized = false;
}

// Fit the splines using tk::spline.
void SwerveOdometryManager::fitSpline() {
    if (measurements.empty()) {
        throw std::runtime_error("No measurements to fit spline.");
    }
    std::vector<double> times;
    std::vector<double> x_positions, y_positions, theta_positions;
    
    double x = 0.0, y = 0.0, theta = 0.0;
    
    // Process each measurement sequentially.
    for (const auto &m : measurements) {
        if(m.angles.size() == 0)
            continue;
        double sum_vx = 0.0, sum_vy = 0.0, sum_omega = 0.0;
        for (int i = 0; i < m.angles.size(); i++) {
            const double angle = m.angles(i);
            std::cout << angle << ",";
            const double distance = m.distances(i);
            sum_vx += distance * std::cos(angle);
            sum_vy += distance * std::sin(angle);
            
            double wheel_y = wheel_base_y/2 * ((i % 2 == 0) ? 1 : -1);
            double wheel_x = wheel_base_x/2 * ((i < 2) ? 1 : -1);

            // std::cout << "Wheel: " << (wheel_x > 0 ? "F" : "B") <<(wheel_y > 0 ? "L" : "R")
            //           << ": " << wheel_x << "," << wheel_y << std::endl;
          
            sum_omega += (wheel_x * std::cos(angle) - wheel_y * std::sin(angle)) * distance 
                       / (wheel_base_x * wheel_base_x + wheel_base_y * wheel_base_y);

        }
        std::cout << std::endl;

        double n = m.angles.size();
        double vx = sum_vx / n;
        double vy = sum_vy / n;
        double omega = (sum_omega / n) / wheel_base_x;
        
        x += vx;
        y += vy;
        theta += omega;
        
        times.push_back(m.time);
        x_positions.push_back(x);
        y_positions.push_back(y);
        theta_positions.push_back(theta);
    }
    
    // Use tk::spline for interpolation.
    spline_x.set_points(times, x_positions);
    spline_y.set_points(times, y_positions);
    spline_theta.set_points(times, theta_positions);
    splinesInitialized = true;
}

// Get the interpolated pose at a given time.
Eigen::Matrix4d SwerveOdometryManager::getPoseAtTime(double t) {
  if (!splinesInitialized) {
    fitSpline();
  }
  double x = spline_x(t);
  double y = spline_y(t);
  double theta = spline_theta(t);
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 0) = cos_theta;
  pose(0, 1) = -sin_theta;
  pose(0, 3) = x;
  pose(1, 0) = sin_theta;
  pose(1, 1) = cos_theta;
  pose(1, 3) = y;
  return pose;
}

std::vector<Eigen::Matrix4d> SwerveOdometryManager::pollSplines(int n_pts) {
  if (measurements.empty())
    throw std::runtime_error("No measurements to poll splines.");
  if (!splinesInitialized)
    fitSpline();
  
  double t_start = measurements.front().time;
  double t_end = measurements.back().time;

  std::vector<Eigen::Matrix4d> result;

  Eigen::MatrixXd poses(n_pts, 2);
  for (int i = 0; i < n_pts; i++) {
    double t = t_start + i * (t_end - t_start) / (n_pts - 1);
    Eigen::Matrix4d pose = getPoseAtTime(t);
    result.push_back(pose);
  }
  
  return result;
}



}

