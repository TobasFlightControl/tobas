#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
class SimpleParamClient
{
public:
  using SharedPtr = std::shared_ptr<SimpleParamClient>;

  inline explicit SimpleParamClient(rclcpp::Node::SharedPtr node, const std::string& node_name)
    : node_(node), node_name_(node_name), client_(node, node_name)
  {
  }

  template <typename T, typename RepType = int64_t, typename DurType = std::ratio<1L>>
  bool setParam(
    const std::string& param_name,
    const T& value,
    std::chrono::duration<RepType, DurType> timeout = std::chrono::seconds(0))
  {
    if (!client_.service_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << node_name_ << "\" parameter server is not ready.");
      return false;
    }

    auto future = client_.set_parameters({ rclcpp::Parameter(param_name, value) });

    if (timeout.count() > 0)
    {
      if (future.wait_for(timeout) == std::future_status::timeout)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before setting \"" << param_name << "\".");
        return false;
      }
    }
    else
    {
      future.wait();
    }

    const auto results = future.get();
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
  const std::string node_name_;

  // 非同期のパラメータクライアント
  // 同期版はspinするからエグゼキュータ上では使えない
  rclcpp::AsyncParametersClient client_;
};
}  // namespace ros2
