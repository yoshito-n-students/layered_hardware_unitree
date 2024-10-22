#include <sstream>
#include <stdexcept>
#include <string>

#include <unitree_actuator_sdk_ros/unitree_actuator_sdk_ros.hpp>

#include <serialPort/SerialPort.h>
#include <unitreeMotor/unitreeMotor.h>

namespace unitree_actuator_sdk_ros {

// =============
// Motor models

MotorType to_motor_type(const std::string &str) {
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

std::string to_string(const MotorType type) {
  switch (type) {
  case MotorType::A1:
    return "A1";
  case MotorType::B1:
    return "B1";
  case MotorType::GO_M8010_6:
    return "GO-M8010-6";
  default:
    std::ostringstream msg;
    msg << "Unknown motor type id (" << static_cast<int>(type) << ")";
    throw std::runtime_error(msg.str());
  }
}

// ======================================
// from / to original sdk (internal use)

static ::MotorType to_original_motor_type(const MotorType type) {
  switch (type) {
  case MotorType::A1:
    return ::MotorType::A1;
  case MotorType::B1:
    return ::MotorType::B1;
  case MotorType::GO_M8010_6:
    return ::MotorType::GO_M8010_6;
  default:
    std::ostringstream msg;
    msg << "Unknown motor type id (" << static_cast<int>(type) << ")";
    throw std::runtime_error(msg.str());
  }
}

static MotorType to_motor_type(const ::MotorType orig_type) {
  switch (orig_type) {
  case ::MotorType::A1:
    return MotorType::A1;
  case ::MotorType::B1:
    return MotorType::B1;
  case ::MotorType::GO_M8010_6:
    return MotorType::GO_M8010_6;
  default:
    std::ostringstream msg;
    msg << "Unknown original motor type id (" << static_cast<int>(orig_type) << ")";
    throw std::runtime_error(msg.str());
  }
}

static ::MotorMode to_original_motor_mode(const MotorMode mode) {
  switch (mode) {
  case MotorMode::BRAKE:
    return ::MotorMode::BRAKE;
  case MotorMode::FOC:
    return ::MotorMode::FOC;
  case MotorMode::CALIBRATE:
    return ::MotorMode::CALIBRATE;
  default:
    std::ostringstream msg;
    msg << "Unknown motor mode id (" << static_cast<int>(mode) << ")";
    throw std::runtime_error(msg.str());
  }
}

static MotorMode to_motor_mode(const ::MotorMode orig_mode) {
  switch (orig_mode) {
  case ::MotorMode::BRAKE:
    return MotorMode::BRAKE;
  case ::MotorMode::FOC:
    return MotorMode::FOC;
  case ::MotorMode::CALIBRATE:
    return MotorMode::CALIBRATE;
  default:
    std::ostringstream msg;
    msg << "Unknown original motor mode id (" << static_cast<int>(orig_mode) << ")";
    throw std::runtime_error(msg.str());
  }
}

// =============
// Serial port

SerialPort::SerialPort(const std::string &port) {
  // open serial port using original sdk
  try {
    orig_ = ErasedTypePtr(new ::SerialPort(port),
                          [](void *const ptr) { delete static_cast<::SerialPort *>(ptr); });
  } catch (const ::IOException &error) {
    // convert original exception to standard one
    throw std::runtime_error(error.what());
  }
}

bool SerialPort::send_recv(const MotorCmd &cmd, MotorData *const data) {
  // get motor type & mode ids for original sdk
  const ::MotorType orig_type = to_original_motor_type(cmd.type);
  const float ratio = ::queryGearRatio(orig_type);
  const ::MotorMode orig_mode = to_original_motor_mode(cmd.mode);

  // build motor command for original sdk, based on given command
  ::MotorCmd orig_cmd;
  orig_cmd.motorType = orig_type;
  orig_cmd.id = cmd.id;
  orig_cmd.mode = ::queryMotorMode(orig_type, orig_mode);
  orig_cmd.kp = cmd.kp;
  orig_cmd.kd = cmd.kd;
  orig_cmd.q = cmd.q * ratio;
  orig_cmd.dq = cmd.dq * ratio;
  orig_cmd.tau = cmd.tau / ratio;

  // create motor state storage for original sdk
  ::MotorData orig_data;
  orig_data.motorType = orig_type;

  // send command & receive state with original sdk
  if (!static_cast<::SerialPort *>(orig_.get())->sendRecv(&orig_cmd, &orig_data)) {
    return false;
  }

  // transfar motor state from original sdk
  data->type = to_motor_type(orig_data.motorType);
  data->id = orig_data.motor_id;
  data->q = orig_data.q / ratio;
  data->dq = orig_data.dq / ratio;
  data->tau = orig_data.tau * ratio;
  data->temp = orig_data.temp;
  data->merror = orig_data.merror;

  return true;
}

} // namespace unitree_actuator_sdk_ros