#ifndef LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_unitree/operating_mode_interface.hpp>
#include <layered_hardware_unitree/unitree_actuator_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <unitree_actuator_sdk_ros/unitree_actuator_sdk_ros.hpp>

namespace layered_hardware_unitree {

class VelocityMode : public OperatingModeInterface {
public:
  VelocityMode(const std::shared_ptr<UnitreeActuatorContext> &context)
      : OperatingModeInterface("velocity", context) {}

  virtual void starting() override { context_->vel_cmd = 0.; }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do because reading states from the actuator is actually performed in write().
    // this is because of limitation of unitree_sdk.
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // pack velocity command.
    // if the command is NaN, use present velocity instead.
    // if the velocity is NaN, use 0.0 instead.
    uasr::MotorCmd cmd{context_->motor_type, context_->id, uasr::MotorMode::FOC};
    cmd.dq = (!std::isnan(context_->vel_cmd) ? context_->vel_cmd : 0.);
    cmd.kd = (!std::isnan(context_->vel_gain) ? context_->vel_gain : 0.);
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
    uasr::MotorCmd cmd{context_->motor_type, context_->id, uasr::MotorMode::FOC};
    uasr::MotorData data{context_->motor_type, context_->id};
    context_->serial->send_recv(cmd, &data);
  }
};

} // namespace layered_hardware_unitree

#endif
