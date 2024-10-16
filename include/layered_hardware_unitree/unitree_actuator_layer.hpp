#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_LAYER_HPP

#include <memory>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_unitree/common_namespaces.hpp>
#include <layered_hardware_unitree/logging_utils.hpp>
#include <layered_hardware_unitree/unitree_actuator.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <serialPort/SerialPort.h>
#include <yaml-cpp/yaml.h>

namespace layered_hardware_unitree {

class UnitreeActuatorLayer : public lh::LayerInterface {
public:
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized =
        lh::LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // find parameter group for this layer
    const auto params_it = hardware_info.hardware_parameters.find(layer_name);
    if (params_it == hardware_info.hardware_parameters.end()) {
      LHU_ERROR("UnitreeActuatorLayer::on_init(): \"%s\" parameter is missing", layer_name.c_str());
      return CallbackReturn::ERROR;
    }

    // parse parameters for this layer as yaml
    std::string serial_iface;
    std::vector<std::string> ator_names;
    std::vector<YAML::Node> ator_params;
    try {
      const YAML::Node params = YAML::Load(params_it->second);
      serial_iface = params["serial_interface"].as<std::string>("/dev/ttyUSB0");
      for (const auto &name_param_pair : params["actuators"]) {
        ator_names.emplace_back(name_param_pair.first.as<std::string>());
        ator_params.emplace_back(name_param_pair.second);
      }
    } catch (const YAML::Exception &error) {
      LHU_ERROR("UnitreeActuatorLayer::on_init(): %s (on parsing \"%s\" parameter)", //
                error.what(), layer_name.c_str());
      return CallbackReturn::ERROR;
    }

    // open USB serial device
    std::shared_ptr<SerialPort> serial;
    try {
      serial.reset(new SerialPort(serial_iface));
    } catch (IOException &e) {
      LHU_ERROR("UnitreeActuatorLayer::on_init(): Failed to open SerialPort: %s", e.what());
      return CallbackReturn::ERROR;
    }

    // init actuators with param "actuators/<actuator_name>"
    for (std::size_t i = 0; i < ator_names.size(); ++i) {
      try {
        actuators_.emplace_back(new UnitreeActuator(ator_names[i], ator_params[i], serial));
      } catch (const std::runtime_error &error) {
        return CallbackReturn::ERROR;
      }
      LHU_INFO("UnitreeActuatorLayer::init(): Initialized the actuator \"%s\"",
               ator_names[i].c_str());
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export reference to actuator states owned by this layer
    std::vector<hi::StateInterface> ifaces;
    for (const auto &ator : actuators_) {
      ifaces = lh::merge(std::move(ifaces), ator->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export reference to actuator commands owned by this layer
    std::vector<hi::CommandInterface> ifaces;
    for (const auto &ator : actuators_) {
      ifaces = lh::merge(std::move(ifaces), ator->export_command_interfaces());
    }
    return ifaces;
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // any state interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // any command interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*state_interfaces*/,
                    std::vector<hi::LoanedCommandInterface> && /*command_interfaces*/) override {
    // any interfaces has to be imported from other layers because this layer is "source"
  }

  virtual hi::return_type
  prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = lh::merge(result, ator->prepare_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type
  perform_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    // notify controller switching to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = lh::merge(result, ator->perform_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // read from all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = lh::merge(result, ator->read(time, period));
    }
    return result;
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // write to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = lh::merge(result, ator->write(time, period));
    }
    return result;
  }

private:
  std::vector<std::unique_ptr<UnitreeActuator>> actuators_;
};

} // namespace layered_hardware_unitree
#endif