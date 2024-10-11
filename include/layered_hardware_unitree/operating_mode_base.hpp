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

protected:
  const std::string name_;
  const UnitreeActuatorDataPtr data_;
};

typedef std::shared_ptr< OperatingModeBase > OperatingModePtr;
typedef std::shared_ptr< const OperatingModeBase > OperatingModeConstPtr;

} // namespace layered_hardware_unitree

#endif // LAYERED_HARDWARE_UNITREE_OPERATING_MODE_BASE_HPP