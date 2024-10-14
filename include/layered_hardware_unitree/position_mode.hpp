#ifndef LAYERED_HARDWARE_UNITREE_POSITION_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_POSITION_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_unitree/operating_mode_interface.hpp>
#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <layered_hardware_unitree/unitree_sdk_helpers.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_unitree {

class PositionMode : public OperatingModeInterface {
public:
  PositionMode(const std::shared_ptr<UnitreeActuatorData> &data)
      : OperatingModeInterface("position", data) {}

  virtual void starting() override {
    // fetch present position and use it as initial value of command
    MotorCmd cmd = initialized_motor_cmd(data_->motor_type, data_->id, MotorMode::BRAKE);
    MotorData data = initialized_motor_data(data_->motor_type);
    data_->serial->sendRecv(&cmd, &data);
    data_->pos_cmd = data.q / queryGearRatio(data_->motor_type);
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do because reading states from the actuator is actually performed in write().
    // this is because of limitation of unitree_sdk.
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    const float ratio = queryGearRatio(data_->motor_type);
    // pack position command.
    // if the command is NaN, use present position instead.
    // if the position is NaN, use 0.0 instead.
    MotorCmd cmd = initialized_motor_cmd(data_->motor_type, data_->id, MotorMode::FOC);
    cmd.q = (!std::isnan(data_->pos_cmd) ? data_->pos_cmd
                                         : (!std::isnan(data_->pos) ? data_->pos : 0.)) *
            ratio;
    cmd.kp = (!std::isnan(data_->pos_gain) ? data_->pos_gain : 0.);
    // pack state data
    MotorData data = initialized_motor_data(data_->motor_type);
    // send & receive (TODO: check the return value)
    data_->serial->sendRecv(&cmd, &data);
    // update state values according to received data
    data_->pos = data.q / ratio;
    data_->vel = data.dq / ratio;
    data_->eff = data.tau;
    data_->temperature = data.temp;
  }

  virtual void stopping() override {
    // disable torque by sending zero command
    MotorCmd cmd = initialized_motor_cmd(data_->motor_type, data_->id, MotorMode::FOC);
    MotorData data = initialized_motor_data(data_->motor_type);
    data_->serial->sendRecv(&cmd, &data);
  }
};
} // namespace layered_hardware_unitree

#endif