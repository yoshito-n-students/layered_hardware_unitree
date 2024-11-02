# layered_hardware_unitree
A ros2_control layer implementation for Unitree actuators. See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware/tree/jazzy) to understand the layered scheme.

## Plugins: layered_hardware_unitree_plugins
### layered_hardware_unitree/UnitreeActuatorLayer
* sends commands to Unitree actuators within `write()` function
* fetches states of actuators within `read()` function
* switches actuators' operation modes within `perform_command_mode_swtich()` function when controllers using associated interfaces activate

#### Hardware parameters
___<layer_name>___ (yaml, required)
* map of parameter names and values for this layer

___<layer_name>.serial_interface___ (string, default: '/dev/ttyUSB0')
* path to USB-serial converter for Unitree actuators

___<layer_name>.actuators___ (map, required)
* map of parameters for each actuator

___<layer_name>.actuators.<actuator_name>.id___ (int, required)
* id of the Unitree actuator

___<layer_name>.actuators.<actuator_name>.motor_type___ (string, required)
* actuator's type name like 'A1', 'B1', & 'GO-M8010-6'

___<layer_name>.actuators.<actuator_name>.pos_gain___ (double, default: 0.1)
* control gain for actuator position (Kp)

___<layer_name>.actuators.<actuator_name>.vel_gain___ (double, default: 0.05)
* control gain for actuator velocity (Kd)

___<layer_name>.actuators.<actuator_name>.operating_mode_map___ (map, required)
* map to actuator's operation mode names from associated interface names (typically joint interfaces)
* possible operation mode names are 'brake', 'position', 'torque' & 'velocity'

#### Example of parameter description
```xml
<param name="example_unitree_actuator_layer">
    serial_interface: /dev/serial/by-id/...
    actuators:
        example_unitree_1:
            id: 1
            motor_type: GO-M8010-6
            operating_mode_map:
                example_joint_1/position: position
                ...
        example_unitree_2:
            id: 2
            ...
</param>
```

## Example
see [examples](layered_hardware_unitree/examples)