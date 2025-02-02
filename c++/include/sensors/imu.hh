#pragma once

#include "sensors/sensor_manager.hh"

#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

namespace frc_sysid {

class IMUManager : public SensorManager {
public:

  struct Measurement {
    double time;
    Eigen::Vector3d acceleration; // 3D acceleration vector.
    Eigen::Vector3d gyroscope;    // 3D angular velocity vector.
  };

  // Constructor.
  IMUManager(const RobotConfig &robot_config, const FieldConfig &field_config);

  // Register a new IMU measurement.
  void registerMeasurement(double time,
                            const Eigen::Vector3d &acceleration,
                            const Eigen::Vector3d &gyroscope);

  // Return the latest measurement.
  const Measurement& getLatestMeasurement() const;

  // Return all measurements within the specified time range.
  std::vector<Measurement> getMeasurementsInTimeRange(double start_time, double end_time) const;

private:
  std::vector<Measurement> measurements;
};

}