#ifndef LAYERED_HARDWARE_UNITREE_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_UNITREE_LOGGING_UTILS_HPP

#include <rclcpp/logging.hpp>

#define LHU_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("layered_hardware_unitree"), __VA_ARGS__)
#define LHU_INFO(...) RCLCPP_INFO(rclcpp::get_logger("layered_hardware_unitree"), __VA_ARGS__)
#define LHU_WARN(...) RCLCPP_WARN(rclcpp::get_logger("layered_hardware_unitree"), __VA_ARGS__)
#define LHU_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("layered_hardware_unitree"), __VA_ARGS__)
#define LHU_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("layered_hardware_unitree"), __VA_ARGS__)

#endif