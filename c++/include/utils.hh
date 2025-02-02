#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

namespace frc_sysid {

std::vector<std::vector<double>> load_csv(const std::string &filename);

}