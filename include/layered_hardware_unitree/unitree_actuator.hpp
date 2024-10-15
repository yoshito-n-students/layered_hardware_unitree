#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_unitree/brake_mode.hpp>
#include <layered_hardware_unitree/common_namespaces.hpp>
#include <layered_hardware_unitree/logging_utils.hpp>
#include <layered_hardware_unitree/operating_mode_interface.hpp>
#include <layered_hardware_unitree/position_mode.hpp>
#include <layered_hardware_unitree/torque_mode.hpp>
#include <layered_hardware_unitree/unitree_actuator_context.hpp>
#include <layered_hardware_unitree/unitree_sdk_helpers.hpp>
#include <layered_hardware_unitree/velocity_mode.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <serialPort/SerialPort.h>
#include <yaml-cpp/yaml.h>

namespace layered_hardware_unitree {

class UnitreeActuator {
public:
  UnitreeActuator(const std::string &name, const YAML::Node &params,
                  const std::shared_ptr<SerialPort> &serial) {
    // parse parameters for this actuator
    unsigned char id;
    std::string motor_type_str;
    double pos_gain, vel_gain;
    std::vector<std::string> mapped_mode_names;
    try {
      id = static_cast<unsigned char>(params["id"].as<int>());
      motor_type_str = params["motor_type"].as<std::string>();
      pos_gain = params["pos_gain"].as<double>(0.1);
      vel_gain = params["vel_gain"].as<double>(0.05);
      for (const auto &iface_mode_name_pair : params["operating_mode_map"]) {
        bound_interfaces_.emplace_back(iface_mode_name_pair.first.as<std::string>());
        mapped_mode_names.emplace_back(iface_mode_name_pair.second.as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      throw std::runtime_error("Failed to parse parameters for \"" + name +
                               "\" actuator: " + error.what());
    }

    // validate motor type
    MotorType motor_type;
    try {
      motor_type = to_motor_type(motor_type_str);
    } catch (const std::runtime_error &error) {
      throw std::runtime_error("Invalid value of \"motor_type\" parameter for \"" + name +
                               "\" actuator: " + error.what());
    }

    // allocate data structure
    context_.reset(new UnitreeActuatorContext{name, serial, id, motor_type, pos_gain, vel_gain});

    // make operating mode map from interface name to unitree's operating mode
    for (const auto &mode_name : mapped_mode_names) {
      try {
        mapped_modes_.emplace_back(make_operating_mode(mode_name));
      } catch (const std::runtime_error &error) {
        throw std::runtime_error("Invalid value in \"operating_mode_map\" parameter for \"" + name +
                                 "\" actuator: " + error.what());
      }
    }
  }

  virtual ~UnitreeActuator() {
    // finalize the present mode
    switch_operating_modes(/* new_mode = */ nullptr);
  }

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export reference to actuator states owned by this actuator
    std::vector<hi::StateInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff);
    ifaces.emplace_back(context_->name, "temperature", &context_->temperature);
    return ifaces;
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export reference to actuator commands owned by this actuator
    std::vector<hi::CommandInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff_cmd);
    return ifaces;
  }

  hi::return_type prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() <= 1) {
      return hi::return_type::OK;
    } else { // active_bound_ifaces.size() >= 2
      LHU_ERROR("UnitreeActuator::prepare_command_mode_switch(): "
                "Reject mode switching of \"%s\" actuator "
                "because %zd bound interfaces are about to be active",
                context_->name.c_str(), active_bound_ifaces.size());
      return hi::return_type::ERROR;
    }
  }

  hi::return_type perform_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() >= 2) {
      LHU_ERROR("UnitreeActuator::perform_command_mode_switch(): "
                "Could not switch mode of \"%s\" actuator "
                "because %zd bound interfaces are active",
                context_->name.c_str(), bound_interfaces_.size());
      return hi::return_type::ERROR;
    }

    // switch to actuator command mode associated with active bound interface
    if (!active_bound_ifaces.empty()) { // active_bound_ifaces.size() == 1
      switch_operating_modes(mapped_modes_[active_bound_ifaces.front()]);
    } else { // active_bound_ifaces.size() == 0
      switch_operating_modes(nullptr);
    }
    return hi::return_type::OK;
  }

  hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
    return hi::return_type::OK; // TODO: return result of read
  }

  hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
    return hi::return_type::OK; // TODO: return result of write
  }

private:
  std::shared_ptr<OperatingModeInterface> make_operating_mode(const std::string &mode_str) const {
    if (mode_str == "brake") {
      return std::make_shared<BrakeMode>(context_);
    } else if (mode_str == "position") {
      return std::make_shared<PositionMode>(context_);
    } else if (mode_str == "velocity") {
      return std::make_shared<VelocityMode>(context_);
    } else if (mode_str == "torque") {
      return std::make_shared<TorqueMode>(context_);
    } else {
      throw std::runtime_error("Unknown operating mode name \"" + mode_str + "\"");
    }
  }

  void switch_operating_modes(const std::shared_ptr<OperatingModeInterface> &new_mode) {
    // do nothing if no mode switch is requested
    if (present_mode_ == new_mode) {
      return;
    }
    // stop present mode
    if (present_mode_) {
      LHU_INFO("UnitreeActuator::switch_operating_modes(): "
               "Stopping \"%s\" operating mode for \"%s\" actuator",
               present_mode_->get_name().c_str(), context_->name.c_str());
      present_mode_->stopping();
      present_mode_.reset();
    }
    // start new mode
    if (new_mode) {
      LHU_INFO("UnitreeActuator::switch_operating_modes(): "
               "Starting \"%s\" operating mode for \"%s\" actuator",
               new_mode->get_name().c_str(), context_->name.c_str());
      new_mode->starting();
      present_mode_ = new_mode;
    }
  }

private:
  std::shared_ptr<UnitreeActuatorContext> context_;

  // present operating mode
  std::shared_ptr<OperatingModeInterface> present_mode_;
  // map from command interface to operating mode
  // (i.e. mapped_modes_[i] is associated with bound_interfaces_[i])
  std::vector<std::string> bound_interfaces_;
  std::vector<std::shared_ptr<OperatingModeInterface>> mapped_modes_;
};

} // namespace layered_hardware_unitree

#endif