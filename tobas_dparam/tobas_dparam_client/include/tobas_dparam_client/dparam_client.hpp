#pragma once

#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

namespace dparam
{
class DynamicParamClient
{
public:
  using SharedPtr = std::shared_ptr<DynamicParamClient>;

  enum Error
  {
    kNoError = 0,
    kServiceNotReady = -1,
    kServerError = -2,
  };

  explicit DynamicParamClient(rclcpp::Node::SharedPtr node, const std::string& node_name, const std::string& ns = "");

  Error setBool(const std::string& param_name, const bool& value);
  Error setInt(const std::string& param_name, const long& value);
  Error setDouble(const std::string& param_name, const long& value);
  Error setString(const std::string& param_name, const std::string& value);

  Error errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string node_name_;
  const std::string ns_;

  Error error_code_ = kNoError;

  template <typename SrvType, const char* SrvName, typename T>
  Error setParam(const std::string& param_name, T& value);
};

template <typename SrvType, const char* SrvName, typename T>
DynamicParamClient::Error DynamicParamClient::setParam(const std::string& param_name, T& value)
{
  ros2::SyncServiceClient<SrvType> sc(node_, path::join(ns_, SrvName));

  const auto req = std::make_shared<typename SrvType::Request>();
  req->node_name = node_name_;
  req->param_name = param_name;
  req->value = value;

  if (!sc.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}
}  // namespace dparam
