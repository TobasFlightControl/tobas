#include "tobas_rviz_plugin/exceptions.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "tobas_rviz_plugin/logger.hpp"

#define LOGGER_NAME "tobas_exception"

namespace tobas
{
ConstructException::ConstructException(const std::string& what_arg) : std::runtime_error(what_arg)
{
  RCLCPP_ERROR_STREAM(
    getLogger(LOGGER_NAME), "Error during construction of object: " << what_arg << "\nException thrown.");
}

Exception::Exception(const std::string& what_arg) : std::runtime_error(what_arg)
{
  RCLCPP_ERROR_STREAM(getLogger(LOGGER_NAME), what_arg << "\nException thrown.");
}
}  // namespace tobas
