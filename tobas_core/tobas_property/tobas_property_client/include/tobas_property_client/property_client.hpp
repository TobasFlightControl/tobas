#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_property_common/constants.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

namespace ptree
{
/**
 * @brief プロパティサーバのクライアント．
 * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
 */
class PropertyClient
{
public:
  using SharedPtr = std::shared_ptr<PropertyClient>;

  enum Error
  {
    kNoError = 0,
    kServiceNotReady = -1,
    kOutputRange = -2,
    kServerError = -3,
  };

  explicit PropertyClient(rclcpp::Node::SharedPtr node, const std::string& section = "DEFAULT");

  Error get(const std::string& key, bool& value);
  Error get(const std::string& key, int& value);
  Error get(const std::string& key, double& value);
  Error get(const std::string& key, std::string& value);

  Error get(const std::string& key, uint8_t& value);
  Error get(const std::string& key, uint16_t& value);
  Error get(const std::string& key, float& value);

  Error set(const std::string& key, const bool& value);
  Error set(const std::string& key, const int& value);
  Error set(const std::string& key, const double& value);
  Error set(const std::string& key, const std::string& value);

  Error set(const std::string& key, const uint8_t& value);
  Error set(const std::string& key, const uint16_t& value);
  Error set(const std::string& key, const float& value);

  Error save();

  Error errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string section_;

  Error error_code_ = kNoError;
  std::string server_error_msg_;

  template <typename SrvType, const char* SrvName, typename T>
  Error getProperty(const std::string& key, T& value);

  template <typename SrvType, const char* SrvName, typename T>
  Error setProperty(const std::string& key, T& value);
};

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::Error PropertyClient::getProperty(const std::string& key, T& value)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get property requested: " << key);

  ros2::SyncServiceClient<SrvType> sc(node_, SrvName);

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;

  if (!sc.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  value = res->value;

  return error_code_ = kNoError;
}

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::Error PropertyClient::setProperty(const std::string& key, T& value)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Set property requested: " << key << ", " << value);

  ros2::SyncServiceClient<SrvType> sc(node_, SrvName);

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;
  req->value = value;

  if (!sc.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}
}  // namespace ptree
