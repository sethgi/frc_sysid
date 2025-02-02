#include "sensors/apriltag.hh"
#include <algorithm>
#include <stdexcept>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

namespace frc_sysid {

using Measurement = AprilTagManager::Measurement;

double rad2deg(double radians) {
  return radians * 180.0 / std::numbers::pi;
}

// Constructor: sets T_world_target to identity.
AprilTagManager::AprilTagManager(const RobotConfig &robot_config, 
                                 const FieldConfig &field_config)
    : SensorManager(robot_config, field_config)
{
}

// Register a new measurement.
void AprilTagManager::registerMeasurement(double time, int tag_id, const Eigen::VectorXd &pose_vec)
{
    // Only register if pose_vec is not (near) zero.
    if (!pose_vec.isZero(1e-9)) {
        Eigen::Matrix4d T_world_robot = Exp(pose_vec);

        Measurement meas { time, tag_id, T_world_robot };
        auto it = std::lower_bound(measurements.begin(), measurements.end(), meas,
            [](const Measurement &a, const Measurement &b) {
                return a.time < b.time;
            });
        measurements.insert(it, meas);
    }
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

// Get a matrix stacking the translation (x, y, z) of each measurement in world frame.
Eigen::MatrixXd AprilTagManager::getTranslations() const {
    auto world_measurements = getMeasurementsWorldFrame();
    int n = static_cast<int>(world_measurements.size());
    Eigen::MatrixXd translations(n, 3);
    for (int i = 0; i < n; i++) {
        translations.row(i) = world_measurements[i].transform.block<3,1>(0,3).transpose();
    }
    return translations;
}


// Return measurements transformed into the world frame.
std::vector<Measurement> AprilTagManager::getMeasurementsWorldFrame() const {
    return measurements;
}

// Return measurements transformed into the robot frame.
std::vector<Measurement> AprilTagManager::getMeasurementsRobotFrame() const {
    std::vector<Measurement> result;
    for (const auto &m : measurements) {
        Measurement out;
        out.time = m.time;
        out.tag_id = m.tag_id;
        out.transform = m.transform.inverse();
        result.push_back(out);
    }
    return result;
}


}