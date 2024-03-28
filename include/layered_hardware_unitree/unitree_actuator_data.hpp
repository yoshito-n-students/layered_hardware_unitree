#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_DATA_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_DATA_HPP

#include <memory>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace layered_hardware_unitree {

struct UnitreeActuatorData {
  UnitreeActuatorData(const std::string &_name, SerialPort *const _serial,
                      const std::uint8_t _id, const MotorType &_motor_type, 
                      const std::vector<double> &_torque_limits, const int &_temp_limit,
                      const double &_pos_gain, const double &_vel_gain)
      : name(_name), m_cmd(), m_data(), serial(_serial), id(_id),
        torque_limits(_torque_limits), 
        motor_type(_motor_type), temp_limit(_temp_limit),
        pos_gain(_pos_gain), vel_gain(_vel_gain),
        pos(0.), vel(0.), eff(0.), temperature(0), pos_cmd(0.), vel_cmd(0.), eff_cmd(0.) {
    // initialize motor command & state
    m_cmd.motorType = motor_type;
    m_cmd.mode = queryMotorMode(motor_type, MotorMode::BRAKE);
    m_cmd.id = id;

    // Torque Off
    m_cmd.tau = 0.;
    m_cmd.dq = 0.;
    m_cmd.q = 0.;
    m_cmd.kp = 0.;
    m_cmd.kd = 0.;

    m_data.motorType = motor_type;
  }


  // handles
  const std::string name;
  MotorCmd m_cmd;
  MotorData m_data;
  SerialPort *const serial;
  const std::uint8_t id;

  // params
  MotorType motor_type;
  const int temp_limit;
  const std::vector<double> torque_limits;
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