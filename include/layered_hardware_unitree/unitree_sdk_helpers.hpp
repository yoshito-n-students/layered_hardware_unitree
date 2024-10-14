#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_SDK_HELPERS
#define LAYERED_HARDWARE_UNITREE_UNITREE_SDK_HELPERS

#include <stdexcept>
#include <string>

#include <unitreeMotor/unitreeMotor.h>

namespace layered_hardware_unitree {

static inline MotorType to_motor_type(const std::string &str) {
  if (str == "GO-M8010-6") {
    return MotorType::GO_M8010_6;
  } else if (str == "A1") {
    return MotorType::A1;
  } else if (str == "B1") {
    return MotorType::B1;
  } else {
    throw std::runtime_error("toMotorType(): Invalied motor type string \"" + str + "\"");
  }
}

static inline MotorCmd initialized_motor_cmd(const MotorType &motor_type, const unsigned short id,
                                             const MotorMode &mode) {
  MotorCmd cmd;
  cmd.motorType = motor_type;
  cmd.id = id;
  cmd.mode = queryMotorMode(motor_type, mode);
  cmd.kp = 0.;
  cmd.kd = 0.;
  cmd.q = 0.;
  cmd.dq = 0.;
  cmd.tau = 0.;
  return cmd;
}

static inline MotorData initialized_motor_data(const MotorType &motor_type) {
  MotorData data;
  data.motorType = motor_type;
  return data;
}

} // namespace layered_hardware_unitree

#endif