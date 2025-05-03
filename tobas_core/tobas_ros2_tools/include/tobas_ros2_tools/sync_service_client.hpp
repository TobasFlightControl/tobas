#pragma once

#include <rclcpp/rclcpp.hpp>

/* 開発用 */
// #include <std_srvs/srv/empty.hpp>
// using SrvType = std_srvs::srv::Empty;

#include "./future.hpp"

namespace ros2
{
/**
 * @brief 同期サービスクライアント．
 * @note ブロッキングを行うため，リアルタイム性が重要なノードでは使用しないこと．
 */
template <typename SrvType>
class SyncServiceClient
{
  static constexpr auto kWaitForServer = std::chrono::seconds(1);

public:
  using SharedPtr = std::shared_ptr<SyncServiceClient>;

  inline explicit SyncServiceClient(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
    : node_(node)
  {
    client_ = node->create_client<SrvType>(name, rclcpp::ServicesQoS(), group);
  }

  /**
   * @brief サービスを呼び，結果が得られるまで待機する．
   *
   * @param req サービスリクエスト．
   * @param timeout レスポンスが得られるまでのタイムアウト．非正ならば無限待機．
   *
   * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool call(
    const typename SrvType::Request::SharedPtr& req,
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    if (!client_->wait_for_service(kWaitForServer))  // service_is_readyは最初のコールでfalseを返すことが多い
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << client_->get_service_name() << "\" service is not ready.");
      return false;
    }

    auto future = client_->async_send_request(req);
    if (waitForFuture(future, timeout) != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << client_->get_service_name() << "\" response.");
      return false;
    }

    res_ = future.get();

    return true;
  }

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool waitForService(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return client_->wait_for_service(timeout);
  }

  inline typename SrvType::Response::SharedPtr getResponse() const
  {
    return res_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<SrvType>::SharedPtr client_;
  typename SrvType::Response::SharedPtr res_;
};
}  // namespace ros2
