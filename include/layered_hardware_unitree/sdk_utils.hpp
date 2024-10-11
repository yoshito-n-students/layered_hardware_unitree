#ifndef LAYERED_HARDWARE_UNITREE_SDK_UTILS_HPP
#define LAYERED_HARDWARE_UNITREE_SDK_UTILS_HPP

#include <sstream>
#include <stdexcept>
#include <string>

#include <unitreeMotor/unitreeMotor.h>

// conversions between MotorType and std::string

static inline MotorType toMotorType(const std::string &str) {
  if (str == "A1") {
    return MotorType::A1;
  } else if (str == "B1") {
    return MotorType::B1;
  } else if (str == "GO-M8010-6") {
    return MotorType::GO_M8010_6;
  } else {
    throw std::runtime_error("Unknown motor type name \"" + str + "\"");
  }
}

static inline std::string toString(const MotorType type) {
  switch (type) {
  case MotorType::A1:
    return "A1";
  case MotorType::B1:
    return "B1";
  case MotorType::GO_M8010_6:
    return "GO-M8010-6";
  default:
    std::ostringstream msg;
    msg << "Unkown motor type id (" << static_cast< int >(type) << ")";
    throw std::runtime_error(msg.str());
  }
}

// generate MotorCmd & MotorData ready for SerialPort::sendRecv()

static inline MotorCmd initializedMotorCmd(const MotorType type, const unsigned short id,
                                           const MotorMode mode) {
  MotorCmd cmd;
  cmd.motorType = type;
  cmd.id = id;
  cmd.mode = queryMotorMode(type, mode);
  cmd.tau = 0.;
  cmd.dq = 0.;
  cmd.q = 0.;
  cmd.kp = 0.;
  cmd.kd = 0.;
  return cmd;
}

static inline MotorData initializedMotorData(const MotorType type) {
  MotorData data;
  data.motorType = type;
  return data;
}

#endif