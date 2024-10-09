#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_DATA_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_DATA_HPP

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <memory>

namespace layered_hardware_unitree {

struct UnitreeActuatorData {
  UnitreeActuatorData(const std::string &_name, SerialPort *const _serial, const std::uint8_t _id,
                      const MotorType &_motor_type, const std::vector< double > &_torque_limits,
                      const int &_temp_limit, const double &_pos_gain, const double &_vel_gain)
      : name(_name), serial(_serial), id(_id), torque_limits(_torque_limits),
        motor_type(_motor_type), temp_limit(_temp_limit), pos_gain(_pos_gain), vel_gain(_vel_gain),
        pos(0.), vel(0.), eff(0.), temperature(0), pos_cmd(0.), vel_cmd(0.), eff_cmd(0.) {}

  // handles
  const std::string name;
  SerialPort *const serial;
  const std::uint8_t id;

  // params
  MotorType motor_type;
  const int temp_limit;
  const std::vector< double > torque_limits;
  const double pos_gain, vel_gain;

  // states
  double pos, vel, eff;
  int temperature;

  // commands
  double pos_cmd, vel_cmd, eff_cmd;
};

typedef std::shared_ptr< UnitreeActuatorData > UnitreeActuatorDataPtr;
typedef std::shared_ptr< const UnitreeActuatorData > UnitreeActuatorDataConstPtr;

} // namespace layered_hardware_unitree

#endif