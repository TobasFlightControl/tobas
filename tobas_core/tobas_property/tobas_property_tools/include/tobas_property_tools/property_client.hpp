#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_path_tools/join.hpp>

#include "./constants.hpp"

namespace ptree
{
class PropertyClient
{
public:
  enum error_t
  {
    E_NO_ERROR = 0,
    E_FAILED_TO_CONNECT = -1,
    E_FAILED_TO_CALL = -2,
    E_OUT_OF_RANGE = -3,
    E_SERVER_ERROR = -4,
  };

  explicit PropertyClient(
    rclcpp::Node::SharedPtr node,
    const std::string& ns = "",
    const std::string& section = "DEFAULT");

  error_t get(const std::string& key, bool& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t get(const std::string& key, int& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t get(const std::string& key, double& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t get(const std::string& key, std::string& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));

  error_t get(const std::string& key, uint8_t& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t get(const std::string& key, uint16_t& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t get(const std::string& key, float& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));

  error_t set(const std::string& key, const bool& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t set(const std::string& key, const int& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t set(const std::string& key, const double& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t
  set(const std::string& key, const std::string& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));

  error_t set(const std::string& key, const uint8_t& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t set(const std::string& key, const uint16_t& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));
  error_t set(const std::string& key, const float& value, const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));

  error_t save(const rclcpp::Duration& timeout = rclcpp::Duration(-1, 0));

  error_t errorCode() const;
  std::string errorMessage() const;

private:
  rclcpp::Node::SharedPtr node_;
  const std::string ns_;
  const std::string section_;

  error_t error_code_ = E_NO_ERROR;
  std::string server_error_msg_;

  template <typename SrvType, const char* SrvName, typename T>
  error_t getProperty(const std::string& key, T& value, const rclcpp::Duration& timeout);

  template <typename SrvType, const char* SrvName, typename T>
  error_t setProperty(const std::string& key, T& value, const rclcpp::Duration& timeout);
};

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::error_t PropertyClient::getProperty(const std::string& key, T& value, const rclcpp::Duration& timeout)
{
  auto client = node_->create_client<SrvType>(path::join(ns_, SrvName));
  if (!client->wait_for_service(timeout.to_chrono<std::chrono::milliseconds>()))
    return error_code_ = E_FAILED_TO_CONNECT;

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;

  auto id = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, id) != rclcpp::FutureReturnCode::SUCCESS)
    return error_code_ = E_FAILED_TO_CALL;

  const auto res = id.get();
  if (!res->success)
  {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::error_t PropertyClient::setProperty(const std::string& key, T& value, const rclcpp::Duration& timeout)
{
  auto client = node_->create_client<SrvType>(path::join(ns_, SrvName));
  if (!client->wait_for_service(timeout.to_chrono<std::chrono::milliseconds>()))
    return error_code_ = E_FAILED_TO_CONNECT;

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;
  req->value = value;

  auto id = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, id) != rclcpp::FutureReturnCode::SUCCESS)
    return error_code_ = E_FAILED_TO_CALL;

  const auto res = id.get();
  if (!res->success)
  {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}
}  // namespace ptree
