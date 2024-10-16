#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_LAYER_HPP

#include <memory>

#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_unitree/common_namespaces.hpp>
#include <layered_hardware_unitree/controller_set.hpp>
#include <layered_hardware_unitree/unitree_actuator.hpp>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace layered_hardware_unitree {

class UnitreeActuatorLayer : public lh::LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) override {
    // make actuator interfaces registered to the hardware
    // so that other layers can find the interfaces
    makeRegistered< hi::ActuatorStateInterface >(hw);
    makeRegistered< hi::PositionActuatorInterface >(hw);
    makeRegistered< hi::VelocityActuatorInterface >(hw);
    makeRegistered< hi::EffortActuatorInterface >(hw);
    makeRegistered< hie::Int32StateInterface >(hw);

    // open USB serial device
    std::string port = param< std::string >(param_nh, "serial_interface", "/dev/ttyUSB0");
    try {
      serial_ = std::make_shared< SerialPort >(port);
    } catch (IOException &e) {
      ROS_ERROR_STREAM("UnitreeActuatorLayer::init(): Failed to open SerialPort: " << e.what());
      return false;
    }

    // load actuator names from param "actuators"
    XmlRpc::XmlRpcValue ators_param;
    if (!param_nh.getParam("actuators", ators_param)) {
      ROS_ERROR_STREAM("UnitreeActuatorLayer::init(): Failed to get param '"
                       << param_nh.resolveName("actuators") << "'");
      return false;
    }
    if (ators_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM("UnitreeActuatorLayer::init(): Param '" << param_nh.resolveName("actuators")
                                                               << "' must be a struct");
      return false;
    }

    // init actuators with param "actuators/<actuator_name>"
    // (could not use BOOST_FOREACH here to avoid a bug in the library in Kinetic)
    for (const XmlRpc::XmlRpcValue::ValueStruct::value_type &ator_param : ators_param) {
      UnitreeActuatorPtr ator(new UnitreeActuator());
      ros::NodeHandle ator_param_nh(param_nh, ros::names::append("actuators", ator_param.first));
      if (!ator->init(ator_param.first, serial_, hw, ator_param_nh)) {
        return false;
      }
      ROS_INFO_STREAM("UnitreeActuatorLayer::init(): Initialized the actuator '" << ator_param.first
                                                                                 << "'");
      actuators_.push_back(ator);
    }

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) override {
    // dry-update of the list of running controllers
    const ControllerSet updated_list(controllers_.updated(start_list, stop_list));

    // ask to all actuators if controller switching is possible
    for (const UnitreeActuatorPtr &ator : actuators_) {
      if (!ator->prepareSwitch(updated_list)) {
        return false;
      }
    }

    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) override {
    // update the list of running controllers
    controllers_.update(start_list, stop_list);

    // notify controller switching to all actuators
    for (const UnitreeActuatorPtr &ator : actuators_) {
      ator->doSwitch(controllers_);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read from all actuators
    for (const UnitreeActuatorPtr &ator : actuators_) {
      ator->read(time, period);
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write to all actuators
    for (const UnitreeActuatorPtr &ator : actuators_) {
      ator->write(time, period);
    }
  }

private:
  // make an hardware interface registered. the interface must be in the static memory space
  // to allow access from outside of this plugin.
  template < typename Interface > static void makeRegistered(hi::RobotHW *const hw) {
    if (!hw->get< Interface >()) {
      static Interface iface;
      hw->registerInterface(&iface);
    }
  }

private:
  std::shared_ptr< SerialPort > serial_;
  ControllerSet controllers_;
  std::vector< UnitreeActuatorPtr > actuators_;
};

} // namespace layered_hardware_unitree
#endif