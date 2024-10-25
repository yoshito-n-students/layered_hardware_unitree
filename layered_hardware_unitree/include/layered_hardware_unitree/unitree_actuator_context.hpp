#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_CONTEXT_HPP

#include <limits>
#include <memory>
#include <sstream>

#include <unitree_actuator_sdk_ros/unitree_actuator_sdk_ros.hpp>

namespace layered_hardware_unitree {

struct UnitreeActuatorContext {
  // handles
  const std::string name;
  const std::shared_ptr<uasr::SerialPort> serial;
  const unsigned char id;

  // params
  const uasr::MotorType motor_type;
  const double pos_gain, vel_gain;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN(),
         temperature = std::numeric_limits<double>::quiet_NaN();

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();
};

// utility functions

static inline std::string get_display_name(const UnitreeActuatorContext &context) {
  std::ostringstream disp_name;
  disp_name << "\"" << context.name << "\" actuator (id: " << static_cast<unsigned int>(context.id)
            << ")";
  return disp_name.str();
}

} // namespace layered_hardware_unitree

#endif