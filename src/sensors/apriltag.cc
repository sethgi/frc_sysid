#include "sensors/apriltag.hh"
#include <algorithm>
#include <stdexcept>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

namespace frc_sysid {

using Measurement = AprilTagManager::Measurement;

constexpr double rad2deg(double radians) {
  return radians * 180.0 / std::numbers::pi;
}


constexpr double deg2rad(double degrees) {
  return degrees * std::numbers::pi / 180.0;
}


// Constructor: sets T_world_target to identity.
AprilTagManager::AprilTagManager(const RobotConfig &robot_config, 
                                 const FieldConfig &field_config)
    : SensorManager(robot_config, field_config)
{
}

// Register a new measurement.
void AprilTagManager::registerMeasurement(double time, int tag_id, const Eigen::VectorXd &meas_vec)
{
    if(tag_id < 0) return;

    const double azimuth = -meas_vec(0);
    const double elevation = -meas_vec(1);
    const double range = meas_vec(3);

    double azimuth_rad = deg2rad(azimuth);
    double elevation_rad = deg2rad(elevation);

    double x = cos(elevation_rad) * cos(azimuth_rad);
    double y = cos(elevation_rad) * sin(azimuth_rad);
    double z = sin(elevation_rad);

    Measurement result;
    result.time = time;
    result.tag_id = tag_id;
    result.bearing_vec << x, y, z;
    result.range = range;

    measurements.push_back(result);
}

// Compute a transformation matrix from a 6-element pose vector.
Eigen::Matrix4d AprilTagManager::Exp(const Eigen::VectorXd &pose_vec)
{
    if (pose_vec.size() < 6)
        throw std::runtime_error("pose_vec must have at least 6 elements.");
    Eigen::Vector3d t = pose_vec.segment<3>(0);
    Eigen::Vector3d rpy = pose_vec.segment<3>(3);
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(rad2deg(rpy[0]), Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(rad2deg(rpy[1]), Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(rad2deg(rpy[2]), Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    res.block<3,3>(0,0) = rotation;
    res.block<3,1>(0,3) = t;
    return res;
}

// Return measurements transformed into the world frame.
std::vector<Measurement> AprilTagManager::getMeasurementsCameraFrame() const {
    return measurements;
}


}