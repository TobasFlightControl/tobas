#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "../include/tobas_rviz_plugin/exceptions.hpp"
#include "../include/tobas_rviz_plugin/logger.hpp"

namespace tobas
{
ConstructException::ConstructException(const std::string& what_arg) : std::runtime_error(what_arg)
{
  RCLCPP_ERROR(
    getLogger("tobas_exception"), "Error during construction of object: %s\nException thrown.", what_arg.c_str());
}

Exception::Exception(const std::string& what_arg) : std::runtime_error(what_arg)
{
  RCLCPP_ERROR(getLogger("tobas_exception"), "%s\nException thrown.", what_arg.c_str());
}
}  // namespace tobas
