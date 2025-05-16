#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_path_tools/join.hpp>
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

  enum error_t
  {
    E_NO_ERROR = 0,
    E_SERVICE_NOT_READY = -1,
    E_OUT_OF_RANGE = -2,
    E_SERVER_ERROR = -3,
  };

  explicit PropertyClient(
    rclcpp::Node::SharedPtr node,
    const std::string& ns = "",
    const std::string& section = "DEFAULT");

  error_t get(const std::string& key, bool& value);
  error_t get(const std::string& key, int& value);
  error_t get(const std::string& key, double& value);
  error_t get(const std::string& key, std::string& value);

  error_t get(const std::string& key, uint8_t& value);
  error_t get(const std::string& key, uint16_t& value);
  error_t get(const std::string& key, float& value);

  error_t set(const std::string& key, const bool& value);
  error_t set(const std::string& key, const int& value);
  error_t set(const std::string& key, const double& value);
  error_t set(const std::string& key, const std::string& value);

  error_t set(const std::string& key, const uint8_t& value);
  error_t set(const std::string& key, const uint16_t& value);
  error_t set(const std::string& key, const float& value);

  error_t save();

  error_t errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string ns_;
  const std::string section_;

  error_t error_code_ = E_NO_ERROR;
  std::string server_error_msg_;

  template <typename SrvType, const char* SrvName, typename T>
  error_t getProperty(const std::string& key, T& value);

  template <typename SrvType, const char* SrvName, typename T>
  error_t setProperty(const std::string& key, T& value);
};

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::error_t PropertyClient::getProperty(const std::string& key, T& value)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get property requested: " << key);

  ros2::SyncServiceClient<SrvType> sc(node_, path::join(ns_, SrvName));

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;

  if (!sc.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  value = res->value;

  return error_code_ = E_NO_ERROR;
}

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::error_t PropertyClient::setProperty(const std::string& key, T& value)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Set property requested: " << key << ", " << value);

  ros2::SyncServiceClient<SrvType> sc(node_, path::join(ns_, SrvName));

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;
  req->value = value;

  if (!sc.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}
}  // namespace ptree
