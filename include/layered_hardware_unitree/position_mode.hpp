#ifndef LAYERED_HARDWARE_UNITREE_POSITION_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_POSITION_MODE_HPP

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
class PositionMode : public OperatingModeBase {
public:
  PositionMode(const UnitreeActuatorDataPtr &data) 
      : OperatingModeBase("position", data) {}
    
  virtual void starting() override {
    data_->m_cmd.kp = 0.01;
    data_->m_cmd.q = 0.;
    sendRecv();
    readAllStates();

    data_->pos_cmd = data_->pos;
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();    
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read pos, vel, eff, temp
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect
    const bool do_write_pos(!std::isnan(data_->pos_cmd) &&
                            areNotEqual(data_->pos_cmd, prev_pos_cmd_));
    if (do_write_pos) {
      if (data_->temp_limit < data_->temperature) {
        data_->m_cmd.q = data_->pos;
      }
      else {
        data_->m_cmd.q = data_->pos_cmd;
      }
      prev_pos_cmd_ = data_->pos_cmd;
    }
    sendRecv();
  }

  virtual void stopping() override { torqueOff(); }

private:
  double prev_pos_cmd_;
};
}

#endif