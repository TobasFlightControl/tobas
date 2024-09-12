#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
class SyncParamClient
{
public:
  using SharedPtr = std::shared_ptr<SyncParamClient>;

  inline explicit SyncParamClient(rclcpp::Node::SharedPtr node, const std::string& remove_node_name)
    : node_(node), remove_node_name_(remove_node_name), client_(node, remove_node_name)
  {
  }

  template <typename T>
  bool setParam(
    const std::string& param_name,
    const T& value,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
  {
    if (!client_.service_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << remove_node_name_ << "\" parameter server is not ready.");
      return false;
    }

    const auto results = client_.set_parameters({ rclcpp::Parameter(param_name, value) }, timeout);
    if (results.size() != 1)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Result size mismatch.");
      return false;
    }

    const auto res = results.front();
    if (!res.successful)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to set \"" << param_name << "\": " << res.reason);
      return false;
    }

    return true;
  }

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string remove_node_name_;

  // 非同期のパラメータクライアント
  // 同期版はspinするからエグゼキュータ上では使えない
  rclcpp::SyncParametersClient client_;
};
}  // namespace ros2
