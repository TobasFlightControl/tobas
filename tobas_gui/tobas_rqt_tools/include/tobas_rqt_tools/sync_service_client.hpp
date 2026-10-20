// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/thread.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

namespace tobas::rqt
{
/** `tobas::ros2::SyncServiceClient`の各処理をQtのメインスレッドを止めずに実行する． */
template <typename SrvType>
class SyncServiceClientWrapper
{
public:
  explicit SyncServiceClientWrapper(
    rclcpp::Node::SharedPtr _node,
    const std::string& _name,
    rclcpp::CallbackGroup::SharedPtr _group = nullptr);

  template <typename RepT = int64_t, typename RatioT = std::milli>
  typename SrvType::Response::ConstSharedPtr sendRequestAndWait(
    const typename SrvType::Request::SharedPtr& _req,
    std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1));

  inline bool serviceIsReady() const;

  template <typename RepT = int64_t, typename RatioT = std::milli>
  inline bool waitForService(std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1));

private:
  ros2::SyncServiceClient<SrvType> impl_;
};

template <typename SrvType>
SyncServiceClientWrapper<SrvType>::SyncServiceClientWrapper(
  rclcpp::Node::SharedPtr _node,
  const std::string& _name,
  rclcpp::CallbackGroup::SharedPtr _group)
  : impl_(_node, _name, _group)
{
}

template <typename SrvType>
template <typename RepT, typename RatioT>
typename SrvType::Response::ConstSharedPtr SyncServiceClientWrapper<SrvType>::sendRequestAndWait(
  const typename SrvType::Request::SharedPtr& _req,
  std::chrono::duration<RepT, RatioT> _timeout)
{
  typename SrvType::Response::ConstSharedPtr res;
  qt::startThreadAndWait([this, &_req, _timeout, &res]() { res = impl_.sendRequestAndWait(_req, _timeout); });
  return res;
}

template <typename SrvType>
inline bool SyncServiceClientWrapper<SrvType>::serviceIsReady() const
{
  return impl_->serviceIsReady();
}

template <typename SrvType>
template <typename RepT, typename RatioT>
inline bool SyncServiceClientWrapper<SrvType>::waitForService(std::chrono::duration<RepT, RatioT> _timeout)
{
  bool res = false;
  qt::startThreadAndWait([this, _timeout, &res]() { res = impl_.waitForService(_timeout); });
  return res;
}
}  // namespace tobas::rqt
