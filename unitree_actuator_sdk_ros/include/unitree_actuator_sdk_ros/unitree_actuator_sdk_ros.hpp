#ifndef UNITREE_ACTUATOR_SDK_ROS_UNITREE_ACTUATOR_SDK_ROS_HPP
#define UNITREE_ACTUATOR_SDK_ROS_UNITREE_ACTUATOR_SDK_ROS_HPP

#include <functional>
#include <memory>
#include <string>

namespace unitree_actuator_sdk_ros {

// =============
// Motor models

enum class MotorType { A1, B1, GO_M8010_6 };

MotorType to_motor_type(const std::string &str);

std::string to_string(const MotorType type);

// ========================
// Motor's operation modes

enum class MotorMode { BRAKE, FOC, CALIBRATE };

// ==============
// Motor command

struct MotorCmd {
  MotorType type;
  unsigned int id;
  MotorMode mode;
  float kp = 0.;
  float kd = 0.;
  // after reduction by the motor's built-in reducer
  float q = 0.;
  float dq = 0.;
  float tau = 0.;
};

// ============
// Motor state

struct MotorData {
  MotorType type;
  unsigned int id;
  // after reduction by the motor's built-in reducer
  float q = 0.;
  float dq = 0.;
  float tau = 0.;
  int temp = 0;
  int merror = 0;
};

// ============
// Serial port

class SerialPort {
public:
  SerialPort(const std::string &port);

  bool send_recv(const MotorCmd &cmd, MotorData *const data);

private:
  std::unique_ptr<void, std::function<void(void *)>> orig_;
};

} // namespace unitree_actuator_sdk_ros

#endif