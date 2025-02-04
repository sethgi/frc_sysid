#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>


namespace frc_sysid {

std::vector<std::vector<double>> load_csv(const std::string &filename);

template <typename T>
concept FloatingPoint = std::floating_point<T> || 
                        (requires(T v) { typename T::Scalar; } && std::floating_point<typename T::Scalar>);

template <FloatingPoint T>
T deg2rad(const T& data) {
  return data * M_PI / 180.;
}

template <FloatingPoint T>
T rad2deg(const T& data) {
  return data * 180. / M_PI;
}

void printPose(const Eigen::Matrix4d& T, const std::string& name);
void printPose(const gtsam::Pose3& P, const std::string& name);


}