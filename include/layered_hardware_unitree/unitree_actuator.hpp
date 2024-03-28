#ifndef LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_HPP
#define LAYERED_HARDWARE_UNITREE_UNITREE_ACTUATOR_HPP

#include <map>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface_extensions/integer_interface.hpp>

#include <layered_hardware_unitree/common_namespaces.hpp>
#include <layered_hardware_unitree/operating_mode_base.hpp>
#include <layered_hardware_unitree/unitree_actuator_data.hpp>
#include <layered_hardware_unitree/controller_set.hpp>
#include <layered_hardware_unitree/position_mode.hpp>
#include <layered_hardware_unitree/velocity_mode.hpp>
#include <layered_hardware_unitree/torque_mode.hpp>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

namespace layered_hardware_unitree {

class UnitreeActuator {
public:
  UnitreeActuator() {}

  virtual ~UnitreeActuator() {
    // finalize the present mode
    if (present_mode_) {
      ROS_WARN_STREAM("UnitreeActuator::~UnitreeActuator(): stopping operating mode '"
                      << present_mode_->getName() << "' for actuator '" << data_->name
                      << "' (id: " << static_cast< int >(data_->id) << ")");
      present_mode_->stopping();
      present_mode_ = OperatingModePtr();
    }
  }

  bool init(const std::string &name, SerialPort *const serial, hi::RobotHW *const hw,
            const ros::NodeHandle &param_nh) {

    // unitree id from param
    int id;
    if (!param_nh.getParam("id", id)) {
      ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("id") << "'");
      return false;
    }

    // unitree motor type
    std::string motor_type;
    if (!param_nh.getParam("motor_type", motor_type)) {
      ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("motor_type") << "'");
      return false;
    }    

    // find unitree actuator by id
    if (!findMotor(id, motor_type, serial)) {
      ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to find the actuator '"
                       << name << "' (id: " << static_cast< int >(id) << ")");
      return false;
    }

    // // torque constant from param
    // double torque_constant;
    // if (!param_nh.getParam("torque_constant", torque_constant)) {
    //   ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to get param '"
    //                    << param_nh.resolveName("torque_constant") << "'");
    //   return false;
    // }

    // get torque limit from parameter
    std::vector<double> torque_limits;
    if (!param_nh.getParam("torque_limits", torque_limits)) {
      ROS_WARN_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("torque_limits") << "' so it has no torque limits");
    }

    // get temperature limit from parameter
    int temp_limit = 0;
    if (!param_nh.getParam("temperature_limit", temp_limit)) {
      ROS_WARN_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("temperature_limit") << "' so it has no temperature limit");
    }

    // get position gain from parameter
    double pos_gain = 0.5;
    if (!param_nh.getParam("pos_gain", pos_gain)) {
      ROS_WARN_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("pos_gain") << "' so it has use default value: " << pos_gain);
    }

    // get velocity gain from parameter
    double vel_gain = 0.01;
    if (!param_nh.getParam("vel_gain", vel_gain)) {
      ROS_WARN_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("vel_gain") << "' so it has use default value: " << vel_gain);
    }

    // allocate data structure
    data_.reset(new UnitreeActuatorData(name, serial, id, getMotorType(motor_type), torque_limits, temp_limit, pos_gain, vel_gain));


    // register actuator states & commands to corresponding hardware interfaces
    const hi::ActuatorStateHandle state_handle(data_->name, &data_->pos, &data_->vel, &data_->eff);
    if (!registerActuatorTo< hi::ActuatorStateInterface >(hw, state_handle) ||
        !registerActuatorTo< hi::PositionActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->pos_cmd)) ||
        !registerActuatorTo< hi::VelocityActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->vel_cmd)) ||
        !registerActuatorTo< hi::EffortActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->eff_cmd))) {
      return false;
    }

    // register actuator temperature state to coressponding hardware interfaces
    if (!registerActuatorTo < hie::Int32StateInterface > (
          hw, hie::Int32StateHandle(data_->name + "/temperature", &data_->temperature))) {
      return false;      
    }    

    // make operating mode map from ros-controller name to unitree's operating mode
    std::map< std::string, std::string > mode_name_map;
    if (!param_nh.getParam("operating_mode_map", mode_name_map)) {
      ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to get param '"
                       << param_nh.resolveName("operating_mode_map") << "'");
      return false;
    }

    for (const std::map< std::string, std::string >::value_type &mode_name : mode_name_map) {
      //
      const std::vector< std::string > controller_names(resolveControllerNames(mode_name.first));
      if (controller_names.empty()) {
        ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to resolve '"
                         << mode_name.first
                         << "' as a controller or controller group name for the actuator '"
                         << data_->name << "' (id: " << static_cast< int >(data_->id) << ")");
        return false;
      }
      //
      const OperatingModePtr mode(makeOperatingMode(mode_name.second));
      if (!mode) {
        ROS_ERROR_STREAM("UnitreeActuator::init(): Failed to make operating mode '"
                         << mode_name.second << "' for the actuator '" << data_->name
                         << "' (id: " << static_cast< int >(data_->id) << ")");
        return false;
      }
      mode_map_[controller_names] = mode;
    }    

    return true;
  }

  bool prepareSwitch(const ControllerSet &controllers) {
    // check if switching is possible by counting number of operating modes after switching
    std::size_t n_modes(0);
    for (const std::map< std::vector< std::string >, OperatingModePtr >::value_type &mode :
         mode_map_) {
      if (controllers.contains(mode.first)) {
        ++n_modes;
        if (n_modes > 1) {
          return false;
        }
      }
    }
    return true;
  }

  void doSwitch(const ControllerSet &controllers) {
    // find the next mode to run by the list of running controllers after switching
    OperatingModePtr next_mode;
    for (const std::map< std::vector< std::string >, OperatingModePtr >::value_type &mode :
         mode_map_) {
      if (controllers.contains(mode.first)) {
        next_mode = mode.second;
        // no more iterations are required because prepareSwitch() ensures
        // there is one match at the maximum
        break;
      }
    }

    // do nothing if no mode switching is required
    if (next_mode == present_mode_) {
      return;
    }

    // switch modes
    if (present_mode_) {
      ROS_INFO_STREAM("UnitreeActuator::doSwitch(): Stopping operating mode '"
                      << present_mode_->getName() << "' for the actuator '" << data_->name
                      << "' (id: " << static_cast< int >(data_->id) << ")");
      present_mode_->stopping();
    }
    if (next_mode) {
      ROS_INFO_STREAM("UnitreeActuator::doSwitch(): Starting operating mode '"
                      << next_mode->getName() << "' for the actuator '" << data_->name
                      << "' (id: " << static_cast< int >(data_->id) << ")");
      next_mode->starting();
    }
    present_mode_ = next_mode;
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
  }

private:
  template < typename Interface, typename Handle >
  static bool registerActuatorTo(hi::RobotHW *const hw, const Handle &handle) {
    Interface *const iface(hw->get< Interface >());
    if (!iface) {
      ROS_ERROR("UnitreeActuator::registerActuatorTo(): Failed to get a hardware interface");
      return false;
    }
    iface->registerHandle(handle);
    return true;
  }

  static std::vector< std::string > resolveControllerNames(const std::string &key) {
    // try resolving the key as a controller group name
    // by searching "<node_ns>/controller_group/<key>"
    {
      std::vector< std::string > val;
      if (ros::param::get(ros::names::append("controller_groups", key), val)) {
        return val;
      }
    }

    // try resolving the key as a controller name
    // by searching "<node_ns>/<key>/type"
    {
      std::string val;
      if (ros::param::get(ros::names::append(key, "type"), val)) {
        return std::vector< std::string >(1, key);
      }
    }

    return std::vector< std::string >();
  }

  static MotorType getMotorType(const std::string &motor_type_str) {
    if (motor_type_str == "GO-M8010-6") {
      return MotorType::GO_M8010_6;
    }
    else if (motor_type_str == "A1") {
      return MotorType::A1;
    }
    else if (motor_type_str == "B1") {
      return MotorType::B1;
    }
    return MotorType();
  }

  static bool findMotor(const int &id, const std::string &motor_type, SerialPort *const serial) {
    MotorCmd cmd;
  
    if (!(motor_type == "GO-M8010-6" || motor_type == "A1" || motor_type == "B1")) {
      ROS_ERROR_STREAM("Unabled motor type: usage(GO-M8010-6, A1, A1)");
      return false;
    }
    
    cmd.motorType = getMotorType(motor_type);
    cmd.mode = queryMotorMode(cmd.motorType, MotorMode::BRAKE);
    cmd.id = id;
    MotorData data;
    data.motorType = getMotorType(motor_type);
    serial->sendRecv(&cmd, &data);
    return data.merror == 0;
  }

  OperatingModePtr makeOperatingMode(const std::string &mode_str) const {
    if (mode_str == "position") {
      return std::make_shared<PositionMode>(data_);
    }
    else if (mode_str == "velocity") {
      return std::make_shared<VelocityMode>(data_);     
    }
    else if (mode_str == "torque") {
      return std::make_shared<TorqueMode>(data_);
    }
    ROS_ERROR_STREAM("UnitreeActuator::makeOperatingMode(): Unknown operating mode name '"
                     << mode_str << " for the actuator '" << data_->name
                     << "' (id: " << static_cast< int >(data_->id) << ")");
    return OperatingModePtr();   
  }

private:
  UnitreeActuatorDataPtr data_;

  std::map< std::vector< std::string >, OperatingModePtr > mode_map_;
  OperatingModePtr present_mode_;
};

typedef std::shared_ptr< UnitreeActuator > UnitreeActuatorPtr;
typedef std::shared_ptr< const UnitreeActuator > UnitreeActuatorConstPtr;

} // namespace layered_hardware_unitree

#endif