#ifndef LAYERED_HARDWARE_UNITREE_BRAKE_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_BRAKE_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_unitree/operating_mode_interface.hpp>
#include <layered_hardware_unitree/unitree_actuator_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <unitree_actuator_sdk_ros/unitree_actuator_sdk_ros.hpp>

namespace layered_hardware_unitree {

class BrakeMode : public OperatingModeInterface {
public:
  BrakeMode(const std::shared_ptr<UnitreeActuatorContext> &context)
      : OperatingModeInterface("brake", context) {}

  virtual void starting() override {
    // nothing to do
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do because reading states from the actuator is actually performed in write().
    // this is because of limitation of unitree_sdk.
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // pack brake command
    const uasr::MotorCmd cmd{context_->motor_type, context_->id, uasr::MotorMode::BRAKE};
    // pack state data
    uasr::MotorData data{context_->motor_type, context_->id};
    // send & receive (TODO: check the return value)
    context_->serial->send_recv(cmd, &data);
    // update state values according to received data
    context_->pos = data.q;
    context_->vel = data.dq;
    context_->eff = data.tau;
    context_->temperature = data.temp;
  }

  virtual void stopping() override {
    // disable torque by sending zero command
    const uasr::MotorCmd cmd{context_->motor_type, context_->id, uasr::MotorMode::FOC};
    uasr::MotorData data{context_->motor_type, context_->id};
    context_->serial->send_recv(cmd, &data);
  }
};
} // namespace layered_hardware_unitree

#endif