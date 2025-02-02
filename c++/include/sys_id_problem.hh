#pragma once

#include "config.hh"
#include "utils.hh"
#include "sensors/apriltag.hh"
#include "sensors/imu.hh"
#include "sensors/odometry.hh"


namespace frc_sysid {

class SysIdProblem {
public:
  SysIdProblem(const RobotConfig &robot_config, const FieldConfig &field_config)
    : robot_config(robot_config), field_config(field_config),
      odometry_manager(robot_config, field_config),
      april_tag_manager(robot_config, field_config),
      imu_manager(robot_config, field_config)
  {}

  // Process the log by loading three CSV files (imu.csv, encoders.csv, tags.csv)
  // from the given directory (base_path). Then register the measurements and
  // compute the world-to-AprilTag transformation.
  void processLog(const std::string &base_path);

  // Accessors to sensor managers.
  SwerveOdometryManager &getOdometryManager() { return odometry_manager; }
  AprilTagManager &getAprilTagManager() { return april_tag_manager; }
  IMUManager &getIMUManager() { return imu_manager; }

private:
  FieldConfig field_config;
  RobotConfig robot_config;
  SwerveOdometryManager odometry_manager;
  AprilTagManager april_tag_manager;
  IMUManager imu_manager;
};

}