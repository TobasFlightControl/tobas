#pragma once

#include <rclcpp/rclcpp.hpp>

#include "./future.hpp"

namespace ros2
{
/**
 * @brief 同期パラメータクライアント．
 * @note ブロッキングを行うため，リアルタイム性が重要なノードでは使用しないこと．
 */
class SyncParamClient
{
  static constexpr auto kWaitForServer = std::chrono::seconds(1);

public:
  using SharedPtr = std::shared_ptr<SyncParamClient>;

  explicit SyncParamClient(rclcpp::Node::SharedPtr node, const std::string& remote_node_name)
    : node_(node), remote_node_name_(remote_node_name), client_(node, remote_node_name)
  {
  }

  template <typename ValueT, typename RepT = int64_t, typename RatioT = std::milli>
  bool setParam(
    const std::string& param_name,
    const ValueT& value,
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    if (!client_.wait_for_service(kWaitForServer))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << remote_node_name_ << "\" parameter server is not ready.");
      return false;
    }

    auto future = client_.set_parameters({ rclcpp::Parameter(param_name, value) });
    if (waitForFuture(future, timeout) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before setting \"" << param_name << "\".");
      return false;
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
  const std::string remote_node_name_;

  // 非同期のパラメータクライアント
  // 同期版はspinするからエグゼキュータ上では使えない
  rclcpp::AsyncParametersClient client_;
};
}  // namespace ros2
