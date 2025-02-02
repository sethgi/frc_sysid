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

#include "config.hh"
#include "sensors/sensor_manager.hh"

namespace frc_sysid {


class AprilTagManager : public SensorManager {
public:
  // Structure to hold an individual AprilTag measurement.
  struct Measurement {
    double time;
    int tag_id;
    Eigen::Matrix4d transform;
  };

  AprilTagManager(const RobotConfig &robot_config, const FieldConfig &field_config);

  void registerMeasurement(double time, int tag_id, const Eigen::VectorXd &pose_vec);

  // Compute a transformation matrix from a 6-element pose vector.
  Eigen::Matrix4d Exp(const Eigen::VectorXd &pose_vec);

  // Get a matrix stacking the translation (x, y, z) of each measurement in world frame.
  Eigen::MatrixXd getTranslations() const;

  Eigen::Matrix4d getTargetPose() const {return T_world_target;}

  // Return measurements transformed into the world frame.
  std::vector<Measurement> getMeasurementsWorldFrame() const;

  // Return measurements transformed into the robot frame (by inverting the relative pose).
  std::vector<Measurement> getMeasurementsRobotFrame() const;

  Measurement getFirstMeasurement() const {
    return measurements.front();
  }

  inline Eigen::Matrix4d getFirstBotPose() const {
    return getMeasurementsWorldFrame().front().transform;
  }

private:
  std::vector<Measurement> measurements;
  Eigen::Matrix4d T_world_target;
};

}