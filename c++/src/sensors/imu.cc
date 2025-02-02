#include "sensors/imu.hh"

#include <stdexcept>

namespace frc_sysid {

// Constructor: Forwards configuration to the SensorManager base class.
IMUManager::IMUManager(const RobotConfig &robot_config, const FieldConfig &field_config)
    : SensorManager(robot_config, field_config)
{}

// Register a new measurement by appending it to the internal vector.
void IMUManager::registerMeasurement(double time,
                                      const Eigen::Vector3d &acceleration,
                                      const Eigen::Vector3d &gyroscope)
{
  Measurement m;
  m.time = time;
  m.acceleration = acceleration;
  m.gyroscope = gyroscope;
  measurements.push_back(m);
}

// Returns the most recent measurement. Throws an exception if none exist.
const IMUManager::Measurement& IMUManager::getLatestMeasurement() const {
  if (measurements.empty())
    throw std::runtime_error("No measurements recorded.");
  return measurements.back();
}

// Returns all measurements whose time falls within the [start_time, end_time] range.
std::vector<IMUManager::Measurement> IMUManager::getMeasurementsInTimeRange(double start_time, double end_time) const {
  std::vector<Measurement> result;
  for (const auto &m : measurements) {
    if(m.time < start_time) continue;
    else if(m.time > end_time) break;

    result.push_back(m);
  }
  return result;
}

}