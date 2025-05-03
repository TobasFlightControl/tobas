#include <rclcpp/rclcpp.hpp>
#include <rsl/random.hpp>

#include "../include/tobas_rviz_plugin/logger.hpp"

namespace tobas
{
/**
 * @brief This is the function that stores the global logger used by Tobas.
 * As it returns a reference to the static logger it can be changed through the `setNodeLoggerName` function.
 */
rclcpp::Logger& getGlobalRootLogger()
{
  static rclcpp::Logger logger = [&]
  {
    // A random number is appended to the name used for the node to make it unique.
    // This unique node and logger name is only used if a user does not set a logger
    // through the `setNodeLoggerName` method to their node's logger.
    auto name = std::format("tobas_{}", rsl::rng()());
    try {
      static auto* tobas_node = new rclcpp::Node(name);
      return tobas_node->get_logger();
    }
    catch (const std::exception& ex) {
      // rclcpp::init was not called so rcl context is null, return non-node logger
      auto logger2 = rclcpp::get_logger(name);
      RCLCPP_WARN_STREAM(logger2, "exception thrown while creating node for logging: " << ex.what());
      RCLCPP_WARN(logger2, "if rclcpp::init was not called, messages from this logger may be missing from /rosout");
      return logger2;
    }
  }();
  return logger;
}

void setNodeLoggerName(const std::string& name)
{
  static auto node = std::make_shared<rclcpp::Node>("tobas", name);
  getGlobalRootLogger() = node->get_logger();
}

rclcpp::Logger getLogger(const std::string& name)
{
  return getGlobalRootLogger().get_child(name);
}
}  // namespace tobas
