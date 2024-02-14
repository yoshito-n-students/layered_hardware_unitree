#ifndef LAYERED_HARDWARE_UNITREE_TORQUE_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_TORQUE_MODE_HPP

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
class TorqueMode : public OperatingModeBase {
public:
  TorqueMode(const UnitreeActuatorDataPtr &data) 
      : OperatingModeBase("torque", data) {}
    
  virtual void starting() override {
    data_->m_cmd.kd = 0.01;
    data_->m_cmd.dq = 0.;
    sendRecv();
    readAllStates();

    data_->eff_cmd = data_->eff;
    prev_eff_cmd_ = std::numeric_limits< double >::quiet_NaN();    
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read pos, vel, eff, temp
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write goal position if the goal pos or profile torque have been updated
    // to make the change affect
    const bool do_write_vel(!std::isnan(data_->eff_cmd) &&
                            areNotEqual(data_->eff_cmd, prev_eff_cmd_));
    if (do_write_vel) {
      if (data_->temp_limit < data_->temperature) {
        data_->m_cmd.tau = 0.;
      }
      else {
        data_->m_cmd.tau = data_->eff_cmd;
      }
      prev_eff_cmd_ = data_->eff_cmd;
    }
    sendRecv();
  }

  virtual void stopping() override { torqueOff(); }

private:
  double prev_eff_cmd_;
};
}

#endif