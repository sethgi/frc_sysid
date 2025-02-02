#include "utils.hh"

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

}