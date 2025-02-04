#include "utils.hh"

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

namespace frc_sysid {

std::vector<std::vector<double>> load_csv(const std::string &filename)
{
  std::vector<std::vector<double>> data;
  std::ifstream file(filename);
  if (!file.is_open())
      throw std::runtime_error("Error opening file: " + filename);

  std::string line;

  if (!std::getline(file, line))
      return data;

  while (std::getline(file, line)) {
    std::vector<double> row;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        row.push_back(std::stod(token));
      } catch (const std::invalid_argument &e) {
        row.push_back(0.0); // Default value on conversion failure.
      }
    }
    data.push_back(row);
  }
  return data;
}

void printPose(const Eigen::Matrix4d& T, const std::string& name)
{
  Eigen::Vector3d translation = T.block<3,1>(0,3);
  Eigen::Matrix3d rotation = T.block<3,3>(0,0);
  Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0); // ZYX convention: yaw, pitch, roll

  
  std::cout << name << ": " << std::endl;
  std::cout << "          Position (x, y, z): " << translation.transpose() << std::endl;
  std::cout << "          Euler angles (ypr, deg): " << rad2deg(euler_angles).transpose() << std::endl;
}

void printPose(const gtsam::Pose3& P, const std::string& name)
{
  const Eigen::Matrix4d T = P.matrix();
  Eigen::Vector3d translation = T.block<3,1>(0,3);
  Eigen::Matrix3d rotation = T.block<3,3>(0,0);
  Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0); // ZYX convention: yaw, pitch, roll

  
  std::cout << name << ": " << std::endl;
  std::cout << "          Position (x, y, z): " << translation.transpose() << std::endl;
  std::cout << "          Euler angles (ypr, deg): " << rad2deg(euler_angles).transpose() << std::endl;
}

}