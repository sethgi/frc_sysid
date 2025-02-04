#pragma once

#include <Eigen/Dense>

namespace frc_sysid {

struct RobotConfig {
  double wheel_radius{2.003 * 0.0254};
  double wheel_base_y{18.75 * 0.0254};
  double wheel_base_x{22.75 * 0.0254};

  Eigen::Matrix4d T_robot_camera;
};

struct FieldConfig {
};


}