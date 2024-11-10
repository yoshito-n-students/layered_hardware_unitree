#ifndef LAYERED_HARDWARE_UNITREE_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_UNITREE_LOGGING_UTILS_HPP

#include <layered_hardware/logging_utils.hpp>
#include <layered_hardware_unitree/common_namespaces.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace layered_hardware_unitree {

// returns reference to the common logger without construction overhead
static inline rclcpp::Logger &get_lhu_logger() {
  static rclcpp::Logger logger = rclcpp::get_logger("layered_hardware_unitree");
  return logger;
}

// logging functions which supports cpp-string arguments

template <typename... Args> static inline void lhu_debug(const char *format, Args &&...args) {
  RCLCPP_DEBUG(get_lhu_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhu_info(const char *format, Args &&...args) {
  RCLCPP_INFO(get_lhu_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhu_warn(const char *format, Args &&...args) {
  RCLCPP_WARN(get_lhu_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhu_error(const char *format, Args &&...args) {
  RCLCPP_ERROR(get_lhu_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhu_fatal(const char *format, Args &&...args) {
  RCLCPP_FATAL(get_lhu_logger(), format, lh::to_format_arg(args)...);
}

} // namespace layered_hardware_dynamixel

#endif