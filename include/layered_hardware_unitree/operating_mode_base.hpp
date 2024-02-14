#ifndef LAYERED_HARDWARE_UNITREE_OPERATING_MODE_BASE_HPP
#define LAYERED_HARDWARE_UNITREE_OPERATING_MODE_BASE_HPP

#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <memory>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_unitree {

class OperatingModeBase {
public:
  OperatingModeBase(const std::string &name, const UnitreeActuatorDataPtr &data) 
      : name_(name), data_(data) {}
    
  virtual ~OperatingModeBase() {}

  std::string getName() const { return name_; }

  // TODO: retrun bool to inform result of mode switching to the upper class
  virtual void starting() = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stopping() = 0;

  void torqueOff() {
    data_->m_cmd.kp = 0.;
    data_->m_cmd.kd = 0.;
    data_->m_cmd.q = 0.;
    data_->m_cmd.dq = 0.;
    data_->m_cmd.tau = 0.;
    sendRecv();
  }

  void sendRecv() {
    data_->serial->sendRecv(&data_->m_cmd, &data_->m_data);
  }

  void readAllStates() {
    data_->pos = data_->m_data.q;
    data_->vel = data_->m_data.dq;
    data_->eff = data_->m_data.tau;
    data_->temperature = data_->m_data.temp;
  }

  //
  // utility
  //

  static bool areNotEqual(const double a, const double b) {
    // does !(|a - b| < EPS) instead of (|a - b| >= EPS) to return True when a and/or b is NaN
    return !(std::abs(a - b) < std::numeric_limits< double >::epsilon());
  }

protected:
  const std::string name_;
  const UnitreeActuatorDataPtr data_;
};

typedef std::shared_ptr< OperatingModeBase > OperatingModePtr;
typedef std::shared_ptr< const OperatingModeBase > OperatingModeConstPtr;

} // namespace layered_hardware_unitree

#endif // LAYERED_HARDWARE_UNITREE_OPERATING_MODE_BASE_HPP