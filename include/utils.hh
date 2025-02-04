#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

namespace frc_sysid {

std::vector<std::vector<double>> load_csv(const std::string &filename);


template <typename T>
T deg2rad(const T& data) {
  return data * std::numbers::pi / 180;
}

template <typename T>
T rad2deg(const T& data) {
  return data * 180 / std::numbers::pi;
}


}