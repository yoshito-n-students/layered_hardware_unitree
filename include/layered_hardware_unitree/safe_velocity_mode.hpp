#ifndef LAYERED_HARDWARE_UNITREE_SAFE_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_UNITREE_SAFE_VELOCITY_MODE_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

#include <layered_hardware_unitree/common_namespaces.hpp>
#include <layered_hardware_unitree/operating_mode_base.hpp>
#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_unitree {
class SafeVelocityMode : public OperatingModeBase {
public:
  SafeVelocityMode(const UnitreeActuatorDataPtr &data) : OperatingModeBase("safe_velocity", data) {}

  virtual void starting() override {
    // initialize motor command & state
    m_cmd.motorType = data_->motor_type;
    m_cmd.mode = queryMotorMode(data_->motor_type, MotorMode::BRAKE);
    m_cmd.id = data_->id;

    // Torque Off
    m_cmd.tau = 0.;
    m_cmd.dq = 0.;
    m_cmd.q = 0.;
    m_cmd.kp = 0.;
    m_cmd.kd = 0.;

    m_data.motorType = data_->motor_type;

    m_cmd.kd = 0.01;
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
          if (data_->torque_limits[i - 1] < abs(data_->eff) &&
              abs(data_->eff) < data_->torque_limits[i]) {
            double scale = (1 / n_limits) /
                               (data_->torque_limits[i - 1] - data_->torque_limits[i]) *
                               (abs(data_->eff) - data_->torque_limits[i - 1]) +
                           (n_limits - i + 1) / n_limits;
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

    const bool do_write_vel(!std::isnan(data_->vel_cmd) && data_->vel_cmd != prev_vel_cmd_ &&
                            !is_limit);
    if (do_write_vel) {

      if (data_->vel_cmd == 0 && !is_stopping_) {
        setPosition(data_->pos);
        is_stopping_ = true;
      } else {
        setVelocity(data_->vel_cmd);
        is_stopping_ = false;
      }
    }

    sendRecv();
    prev_vel_cmd_ = data_->vel_cmd;
  }

  virtual void stopping() override { torqueOff(); }

protected:
  void setCmdZero() {
    m_cmd.kp = 0.;
    m_cmd.kd = 0.;
    m_cmd.q = 0.;
    m_cmd.dq = 0.;
    m_cmd.tau = 0.;
  }

  void torqueOff() {
    setCmdZero();
    sendRecv();
  }

  void setBrakeMode() { m_cmd.mode = queryMotorMode(m_cmd.motorType, MotorMode::BRAKE); }

  void setFOCMode() { m_cmd.mode = queryMotorMode(m_cmd.motorType, MotorMode::FOC); }

  void setPosition(const double &pos, const double &kp) {
    setCmdZero();
    m_cmd.kp = kp;
    m_cmd.q = pos * queryGearRatio(data_->motor_type);
  }

  void setPosition(const double &pos) { setPosition(pos, data_->pos_gain); }

  void setVelocity(const double &vel, const double &kd) {
    setCmdZero();
    m_cmd.kd = kd;
    m_cmd.dq = vel * queryGearRatio(m_cmd.motorType);
  }

  void setVelocity(const double &vel) { setVelocity(vel, data_->vel_gain); }

  bool hasTemperatureLimit() { return data_->temp_limit != 0; }

  bool hasTorqueLimit() { return data_->torque_limits.size() > 1; }

  bool isCommandZero() {
    return m_cmd.kp == 0. && m_cmd.kd == 0. && m_cmd.q == 0. && m_cmd.dq == 0. && m_cmd.tau == 0.;
  }

  void sendRecv() { data_->serial->sendRecv(&m_cmd, &m_data); }

  struct PosStamp {
    double pos;
    ros::Time stamp;
  };

  void readAllStates() {
    data_->pos = m_data.q / queryGearRatio(data_->motor_type);
    // 最小時間10msの角度とタイムスタンプを記録し，記録されたデータから速度を算出する
    static std::vector< PosStamp > pos_stamps;
    pos_stamps.push_back({data_->pos, ros::Time::now()});
    if (pos_stamps.size() > 1 &&
        (pos_stamps.back().stamp - pos_stamps.front().stamp).toSec() > 0.01) {
      data_->vel = (pos_stamps.back().pos - pos_stamps.front().pos) /
                   ((pos_stamps.back().stamp - pos_stamps.front().stamp).toSec());
      pos_stamps.erase(pos_stamps.begin());
    } else if (pos_stamps.size() <= 1) {
      data_->vel = m_data.dq;
    }

    data_->eff = m_data.tau;
    data_->temperature = m_data.temp;
  }

  //
  // utility
  //

  static bool areNotEqual(const double a, const double b) {
    // does !(|a - b| < EPS) instead of (|a - b| >= EPS) to return True when a and/or b is NaN
    return !(std::abs(a - b) < std::numeric_limits< double >::epsilon());
  }

private:
  MotorCmd m_cmd;
  MotorData m_data;
  double prev_vel_cmd_;
  bool is_stopping_;
};
} // namespace layered_hardware_unitree

#endif