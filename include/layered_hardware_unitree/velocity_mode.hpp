#ifndef LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_unitree/operating_mode_interface.hpp>
#include <layered_hardware_unitree/unitree_actuator_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

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
    const float ratio = queryGearRatio(context_->motor_type);
    // pack velocity command.
    // if the command is NaN, use present velocity instead.
    // if the velocity is NaN, use 0.0 instead.
    MotorCmd cmd = initialized_motor_cmd(context_->motor_type, context_->id, MotorMode::FOC);
    cmd.dq = (!std::isnan(context_->vel_cmd) ? context_->vel_cmd : 0.) * ratio;
    cmd.kd = (!std::isnan(context_->vel_gain) ? context_->vel_gain : 0.);
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
