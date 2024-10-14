#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware_unitree/unitree_actuator_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware_unitree::UnitreeActuatorLayer,
                       layered_hardware::LayerInterface);