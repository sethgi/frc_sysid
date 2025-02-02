#pragma once

#include "config.hh"

#include <concepts>
#include <utility> 

namespace frc_sysid {

template<typename T, typename... Args>
concept HasRegisterMeasurement = requires(T t, Args&&... args) {
  { t.registerMeasurement(std::forward<Args>(args)...) } -> std::same_as<void>;
};


class SensorManager {
 public:
    SensorManager(const RobotConfig &robot_config, const FieldConfig &field_config)
      : robot_config(robot_config), field_config(field_config) { }

    virtual ~SensorManager() = default;
    
 protected:
  RobotConfig robot_config;
  FieldConfig field_config;
};

}