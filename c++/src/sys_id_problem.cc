#include "sys_id_problem.hh"

namespace frc_sysid {

void SysIdProblem::processLog(const std::string &base_path)
{
  // Load CSV files.
  auto imu_data = load_csv(base_path + "/imu.csv");         // Header: time, accel_x, accel_y, accel_z, ang_vel_x, ang_vel_y, ang_vel_z
  auto encoders_data = load_csv(base_path + "/encoders.csv");   // Header: time, enc_FL, enc_FR, enc_BL, enc_BR, pos_FL, pos_FR, pos_BL, pos_BR
  auto apriltag_data = load_csv(base_path + "/botposes.csv");           // Header: time, target_x, target_y, target_z, target_roll, target_pitch, target_yaw

  // Process IMU data: register each row.
  for (const auto &row : imu_data) {
    if (row.size() < 7)
      continue;
    double ts = row[0];
    Eigen::Vector3d accel(row[1], row[2], row[3]);
    Eigen::Vector3d ang_vel(row[4], row[5], row[6]);
    imu_manager.registerMeasurement(ts, accel, ang_vel);
  }

  // Process encoder data: first four values after time are wheel angles,
  // next four are wheel distances.
  for (const auto &row : encoders_data) {
    if (row.size() < 9)
      continue;
    double ts = row[0];
    Eigen::VectorXd wheel_angles(4);
    for (int i = 0; i < 4; i++) {
      wheel_angles(i) = row[1 + i];
    }
    Eigen::VectorXd wheel_dist(4);
    for (int i = 0; i < 4; i++) {
      wheel_dist(i) = row[5 + i];
    }
    odometry_manager.registerMeasurement(ts, wheel_angles, wheel_dist);
  }

  // Process tag data: row contains time and then 6 values (target x, y, z and roll, pitch, yaw).
  for (const auto &row : apriltag_data) {
    if (row.size() < 9 || static_cast<int>(row.at(8)) == 0)
      continue;
    const double latency = row[7] / 1000.;
    const double ts = row[0] - latency;

    Eigen::VectorXd tag_vec(6);
    for (int i = 0; i < 6; i++) {
      tag_vec(i) = row[1 + i];
    }
    // The Python code passed "None" for tag id; here we use a dummy id (0).
    april_tag_manager.registerMeasurement(ts, 0, tag_vec);
  }
}




}