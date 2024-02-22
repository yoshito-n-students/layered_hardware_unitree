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
    data_->m_cmd.kd = 0.01;
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
    const bool do_write_vel(!std::isnan(data_->vel_cmd) &&
                            areNotEqual(data_->vel_cmd, prev_vel_cmd_));
    if (do_write_vel) {
      if (data_->temp_limit < data_->temperature) {
        data_->m_cmd.dq = 0.;
      }
      else {
        data_->m_cmd.dq = data_->vel_cmd * queryGearRatio(data_->m_cmd.motorType);
        // ROS_INFO_STREAM("data_->vel_cmd: " << data_->m_cmd.dq);
      }
      prev_vel_cmd_ = data_->vel_cmd;
    }

    // if current velocity & target velocity is 0, set brake mode
    data_->m_cmd.dq == 0. ? setBrakeMode() : setFOCMode();
    sendRecv();
  }

  virtual void stopping() override { torqueOff(); }

private:
  double prev_vel_cmd_;
};
}

#endif