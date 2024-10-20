#pragma once

#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

namespace dparam
{
class DynamicParamClient
{
public:
  using SharedPtr = std::shared_ptr<DynamicParamClient>;

  enum error_t
  {
    E_NO_ERROR = 0,
    E_SERVICE_NOT_READY = -1,
    E_SERVER_ERROR = -2,
  };

  explicit DynamicParamClient(rclcpp::Node::SharedPtr node, const std::string& node_name, const std::string& ns = "");

  error_t set(const std::string& param_name, const bool& value);
  error_t set(const std::string& param_name, const int& value);
  error_t set(const std::string& param_name, const double& value);
  error_t set(const std::string& param_name, const std::string& value);

  error_t errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string node_name_;
  const std::string ns_;

  error_t error_code_ = E_NO_ERROR;

  template <typename SrvType, const char* SrvName, typename T>
  error_t setParam(const std::string& param_name, T& value);
};

template <typename SrvType, const char* SrvName, typename T>
DynamicParamClient::error_t DynamicParamClient::setParam(const std::string& param_name, T& value)
{
  ros2::SyncServiceClient<SrvType> sc(node_, path::join(ns_, SrvName));

  const auto req = std::make_shared<typename SrvType::Request>();
  req->node_name = node_name_;
  req->param_name = param_name;
  req->value = value;

  if (!sc.call(req))
    return error_code_ = E_SERVICE_NOT_READY;

  const auto res = sc.getResponse();
  if (!res->success)
    return error_code_ = E_SERVER_ERROR;

  return error_code_ = E_NO_ERROR;
}
}  // namespace dparam
