#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_DATA_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_DATA_HPP

#include <limits>
#include <memory>

#include <serialPort/SerialPort.h>
#include <unitreeMotor/unitreeMotor.h>

namespace layered_hardware_unitree {

struct UnitreeActuatorData {
  // handles
  const std::string name;
  std::shared_ptr<SerialPort> serial;
  const unsigned char id;

  // params
  const MotorType motor_type;
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

} // namespace layered_hardware_unitree

#endif