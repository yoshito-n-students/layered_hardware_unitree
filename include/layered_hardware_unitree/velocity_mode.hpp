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

    bool is_limit = false;

    // torque limit -------------------------------------------------------------
    // If the torque limit is set and no stop command is currently provided
    if (hasTorqueLimit() && !is_stopping_) {
      // If the torque limit trigger is exceeded
      if (data_->torque_limits[0] < abs(data_->eff)) {
        // Change the scale of the speed command value given to the motor for each 
        // limit between the current torque value and each limit set
        for (int i = 1; i < data_->torque_limits.size(); i++) {
          const int n_limits = data_->torque_limits.size() - 1;
          if (data_->torque_limits[i-1] < abs(data_->eff) && abs(data_->eff) < data_->torque_limits[i]) {
            double scale = (1 / n_limits) / (data_->torque_limits[i-1] - data_->torque_limits[i]) 
                            * (abs(data_->eff) - data_->torque_limits[i-1]) + (n_limits - i + 1) / n_limits;
            setVelocity(data_->vel_cmd * scale);
            break;
          } 
        }

        if (data_->torque_limits.back() < abs(data_->eff)) {
          setVelocity(0.);
        }
        is_limit = true;
      }    
    }
    // --------------------------------------------------------------------------

    // temperature limit --------------------------------------------------------
    // if over the temperature limit, stop the motor
    if (data_->temp_limit < data_->temperature && hasTemperatureLimit()) {
      setVelocity(0.);
      is_limit = true;
    }
    // --------------------------------------------------------------------------

    const bool do_write_vel(!std::isnan(data_->vel_cmd) &&
                            data_->vel_cmd != prev_vel_cmd_ &&
                            !is_limit);
    if (do_write_vel) {

      if (data_->vel_cmd == 0 && !is_stopping_) {
        setPosition(data_->pos);
        is_stopping_ = true;
      }
      else {
        setVelocity(data_->vel_cmd);
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