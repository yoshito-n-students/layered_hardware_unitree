#ifndef LAYERED_HARDWARE_UNITREE_BRAKE_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_BRAKE_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_unitree/operating_mode_interface.hpp>
#include <layered_hardware_unitree/unitree_actuator_context.hpp>
#include <layered_hardware_unitree/unitree_sdk_helpers.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

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
    const float ratio = queryGearRatio(context_->motor_type);
    // pack brake command
    MotorCmd cmd = initialized_motor_cmd(context_->motor_type, context_->id, MotorMode::BRAKE);
    // pack state data
    MotorData data = initialized_motor_data(context_->motor_type);
    // send & receive (TODO: check the return value)
    context_->serial->sendRecv(&cmd, &data);
    // update state values according to received data
    context_->pos = data.q / ratio;
    context_->vel = data.dq / ratio;
    context_->eff = data.tau;
    context_->temperature = data.temp;
  }

  virtual void stopping() override {
    // disable torque by sending zero command
    MotorCmd cmd = initialized_motor_cmd(context_->motor_type, context_->id, MotorMode::FOC);
    MotorData data = initialized_motor_data(context_->motor_type);
    context_->serial->sendRecv(&cmd, &data);
  }
};
} // namespace layered_hardware_unitree

#endif