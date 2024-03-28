#ifndef LAYERED_HARDWARE_UNITREE_OPERATING_MODE_BASE_HPP
#define LAYERED_HARDWARE_UNITREE_OPERATING_MODE_BASE_HPP

#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <memory>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/circular_buffer.hpp>

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

  void setCmdZero() {
    data_->m_cmd.kp = 0.;
    data_->m_cmd.kd = 0.;
    data_->m_cmd.q = 0.;
    data_->m_cmd.dq = 0.;
    data_->m_cmd.tau = 0.;    
  }

  void torqueOff() {
    setCmdZero();
    sendRecv();
  }

  void setBrakeMode() {
    data_->m_cmd.mode = queryMotorMode(data_->m_cmd.motorType, MotorMode::BRAKE);
  }

  void setFOCMode() {
    data_->m_cmd.mode = queryMotorMode(data_->m_cmd.motorType, MotorMode::FOC);
  }

  void setPosition(const double& pos, const double& kp = 0.5) {
    setCmdZero();
    data_->m_cmd.kp = kp;
    data_->m_cmd.q = pos * queryGearRatio(data_->motor_type);
  }

  void setVelocity(const double& vel, const double& kd = 0.01) {
    setCmdZero();
    data_->m_cmd.kd = kd;
    data_->m_cmd.dq = vel * queryGearRatio(data_->m_cmd.motorType);
  }

  // double clampTorque(const double& eff) {

  //   return data_->torque_limit < abs(eff) && hasTorqueLimit() ? 
  //            data_->torque_limit * eff / abs(eff) : eff;
  // }

  void setTorque(const double& eff) {
    setCmdZero();
    data_->m_cmd.tau = eff;
  }

  bool hasTemperatureLimit() {
    return data_->temp_limit != 0;
  }

  bool hasTorqueLimit() {
    return data_->torque_limits.size() > 1;
  }

  bool isCommandZero() {
    return data_->m_cmd.kp == 0. && 
           data_->m_cmd.kd == 0. &&
           data_->m_cmd.q == 0. && 
           data_->m_cmd.dq == 0. &&
           data_->m_cmd.tau == 0.;            
  }

  void sendRecv() {
    data_->serial->sendRecv(&data_->m_cmd, &data_->m_data);
  }

  struct PosStamp {
    double pos;
    ros::Time stamp;
  };

  void readAllStates() {
    data_->pos = data_->m_data.q / queryGearRatio(data_->motor_type);
    // 最小時間10msの角度とタイムスタンプを記録し，記録されたデータから速度を算出する
    static std::vector<PosStamp> pos_stamps;
    pos_stamps.push_back({data_->pos, ros::Time::now()});
    if (pos_stamps.size() > 1 && (pos_stamps.back().stamp - pos_stamps.front().stamp).toSec() > 0.01) {
      data_->vel = (pos_stamps.back().pos - pos_stamps.front().pos) / ((pos_stamps.back().stamp - pos_stamps.front().stamp).toSec());
      pos_stamps.erase(pos_stamps.begin());
    }
    else if (pos_stamps.size() <= 1) {
      data_->vel = data_->m_data.dq;
    }

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