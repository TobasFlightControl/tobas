#pragma once

#include <ros/ros.h>

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
    E_SERVER_NOT_READY = -1,
    E_FAILED_TO_CALL = -2,
    E_OUT_OF_RANGE = -3,
    E_SERVER_ERROR = -4,
  };

  explicit PropertyClient(ros::NodeHandle& nh, const std::string& ns = "", const std::string& section = "DEFAULT");

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
  std::string errorMessage() const;

private:
  ros::NodeHandle& nh_;
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
  ros::ServiceClient client = nh_.serviceClient<SrvType>(path::join(ns_, SrvName));
  if (!client.exists())
    return error_code_ = E_SERVER_NOT_READY;

  SrvType msg;
  msg.request.section = section_;
  msg.request.key = key;

  if (!client.call(msg))
    return error_code_ = E_FAILED_TO_CALL;

  if (!msg.response.success)
  {
    server_error_msg_ = msg.response.message;
    return error_code_ = E_SERVER_ERROR;
  }

  value = msg.response.value;

  return error_code_ = E_NO_ERROR;
}

template <typename SrvType, const char* SrvName, typename T>
PropertyClient::error_t PropertyClient::setProperty(const std::string& key, T& value)
{
  ros::ServiceClient client = nh_.serviceClient<SrvType>(path::join(ns_, SrvName));
  if (!client.exists())
    return error_code_ = E_SERVER_NOT_READY;

  SrvType msg;
  msg.request.section = section_;
  msg.request.key = key;
  msg.request.value = value;

  if (!client.call(msg))
    return error_code_ = E_FAILED_TO_CALL;

  if (!msg.response.success)
  {
    server_error_msg_ = msg.response.message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}
}  // namespace ptree
