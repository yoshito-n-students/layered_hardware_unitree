#ifndef LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP

#include <cmath>

#include <layered_hardware_unitree/operating_mode_base.hpp>
#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_unitree {

class VelocityMode : public OperatingModeBase {
public:
  VelocityMode(const UnitreeActuatorDataPtr &data) : OperatingModeBase("velocity", data) {}

  virtual void starting() override {
    // nothing to do
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // nothing to do as state variables will be updated in write()
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    const float ratio = queryGearRatio(data_->motor_type);
    // build motor command
    // (if command is NaN, use 0 instead)
    MotorCmd cmd = initializedMotorCmd(data_->motor_type, data_->id, MotorMode::FOC);
    cmd.kd = data_->vel_gain;
    cmd.dq = (!std::isnan(data_->vel_cmd) ? data_->vel_cmd : 0.) * ratio;
    // build motor state data
    MotorData data = initializedMotorData(data_->motor_type);
    // send command & receive state
    data_->serial->sendRecv(&cmd, &data);
    // TODO: check data.merror here
    // update state variables
    data_->eff = data.tau;
    data_->vel = data.dq / ratio;
    data_->pos = data.q / ratio;
  }

  virtual void stopping() override {
    // disable torque by sending zero command
    MotorCmd cmd = initializedMotorCmd(data_->motor_type, data_->id, MotorMode::FOC);
    MotorData data = initializedMotorData(data_->motor_type);
    data_->serial->sendRecv(&cmd, &data);
  }
};

} // namespace layered_hardware_unitree

#endif