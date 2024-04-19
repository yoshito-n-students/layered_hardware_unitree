#ifndef LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_VELOCITY_MODE_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

#include <layered_hardware_unitree/common_namespaces.hpp>
#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <layered_hardware_unitree/operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

#include <ros/duration.h>
#include <ros/time.h>

#include <boost/optional.hpp>


namespace layered_hardware_unitree {
class VelocityMode : public OperatingModeBase {
public:
  VelocityMode(const UnitreeActuatorDataPtr &data) 
      : OperatingModeBase("velocity", data) {}
    
  virtual void starting() override {
    data_->m_cmd.kd = data_->vel_gain;
    setFOCMode();
    sendRecv();
    readAllStates();

    data_->vel_cmd = data_->vel;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();    
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read pos, vel, eff, temp
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect

    bool is_torque_limit = false;
    double vel_gain = data_->vel_gain;

    // torque limit -------------------------------------------------------------
    // If the torque limit is set and no stop command is currently provided
    if (hasTorqueLimit() && !is_stopping_) {
      const double approx_input_torque = abs(computeApproximatelyInputTorque(0., data_->vel_cmd, 0.));

      // 1. If the computed torque is larger than torque limit
      if (approx_input_torque > data_->torque_limit) {
        vel_gain = data_->torque_limit / approx_input_torque * data_->vel_gain;
        is_torque_limit = true;
      }

      // 2. If the output torque is larger than torque limit
      if (abs(data_->eff) > data_->torque_limit) {
        vel_gain = data_->torque_limit / abs(data_->eff) * vel_gain;
        is_torque_limit = true;
      }
    }
    // --------------------------------------------------------------------------

    // temperature limit --------------------------------------------------------
    // if over the temperature limit, stop the motor
    if (data_->temp_limit < data_->temperature && hasTemperatureLimit()) {
      setVelocity(0.);
      sendRecv();
      return;
    }
    // --------------------------------------------------------------------------

    const bool do_write_vel(!std::isnan(data_->vel_cmd) &&
                            data_->vel_cmd != prev_vel_cmd_ || is_torque_limit);
    if (do_write_vel) {

      if (data_->vel_cmd == 0 && !is_stopping_) {
        setPosition(data_->pos);
        is_stopping_ = true;
      }
      else {
        setVelocity(data_->vel_cmd, vel_gain);
        is_stopping_ = false;
      }
    }

    sendRecv();
    prev_vel_cmd_ = data_->vel_cmd;
  }

  virtual void stopping() override { torqueOff(); }

private:
  double prev_vel_cmd_;
  bool is_stopping_;
};
}

#endif
