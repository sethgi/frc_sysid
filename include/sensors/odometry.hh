#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <concepts>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// From: https://kluge.in-chemnitz.de/opensource/spline/spline.h
#include "thirdparty/spline.h"
#include "config.hh"
#include "sensors/sensor_manager.hh"

namespace frc_sysid{

class SwerveOdometryManager : public SensorManager {
public:
  SwerveOdometryManager(const RobotConfig &robot_config, const FieldConfig &field_config);

  struct Measurement {
    double time;
    Eigen::VectorXd angles;
    Eigen::VectorXd distances;
  };

  // Register a new measurement. The method computes delta distances
  // and inserts the measurement into a time‑sorted vector.
  void registerMeasurement(double time_since_start,
                            const Eigen::Vector4d &wheel_angles,
                            const Eigen::Vector4d &wheel_dist);

  // Fit the splines using tk::spline. For each coordinate (x, y, theta),
  // we extract the corresponding positions computed from the measurements.
  void fitSpline();

  // Get the interpolated pose at a given time.
  // If se3 is true, a 4x4 homogeneous transform is returned.
  // Otherwise, a 3x3 pose matrix is returned.
  Eigen::Matrix4d getPoseAtTime(double t);

  std::vector<Eigen::Matrix4d> pollSplines(int n_pts);

  const std::vector<Measurement>& getMeasurements() const {
    return measurements;
  }

private:
  double wheel_radius;
  double wheel_base_x;
  double wheel_base_y;
  Eigen::VectorXd prev_dist;
  bool prev_dist_initialized;
  
  std::vector<Measurement> measurements;
  
  // Use tk::spline for each coordinate.
  tk::spline spline_x, spline_y, spline_theta;
  bool splinesInitialized;
};

}