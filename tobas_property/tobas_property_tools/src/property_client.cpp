#include <std_srvs/srv/trigger.hpp>

#include <tobas_property_msgs/srv/get_bool.hpp>
#include <tobas_property_msgs/srv/get_int.hpp>
#include <tobas_property_msgs/srv/get_double.hpp>
#include <tobas_property_msgs/srv/get_string.hpp>
#include <tobas_property_msgs/srv/set_bool.hpp>
#include <tobas_property_msgs/srv/set_int.hpp>
#include <tobas_property_msgs/srv/set_double.hpp>
#include <tobas_property_msgs/srv/set_string.hpp>

#include "../include/tobas_property_tools/property_client.hpp"

using namespace std;
using namespace std_srvs::srv;
using namespace tobas_property_msgs::srv;

namespace ptree
{
PropertyClient::PropertyClient(rclcpp::Node::SharedPtr node, const string& ns, const string& section)
  : node_(node), ns_(ns), section_(section)
{
}

PropertyClient::error_t PropertyClient::get(const string& key, bool& value, const rclcpp::Duration& timeout)
{
  return getProperty<GetBool, kGetBoolSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, int& value, const rclcpp::Duration& timeout)
{
  return getProperty<GetInt, kGetIntSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, double& value, const rclcpp::Duration& timeout)
{
  return getProperty<GetDouble, kGetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, string& value, const rclcpp::Duration& timeout)
{
  return getProperty<GetString, kGetStringSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, uint8_t& value, const rclcpp::Duration& timeout)
{
  int tmp;

  if (getProperty<GetInt, kGetIntSrv>(key, tmp, timeout) < 0)
    return error_code_;

  if (tmp < 0 || UINT8_MAX < tmp)
    return error_code_ = E_OUT_OF_RANGE;

  value = static_cast<uint8_t>(tmp);
  return error_code_ = E_NO_ERROR;
}

PropertyClient::error_t PropertyClient::get(const string& key, uint16_t& value, const rclcpp::Duration& timeout)
{
  int tmp;

  if (getProperty<GetInt, kGetIntSrv>(key, tmp, timeout) < 0)
    return error_code_;

  if (tmp < 0 || UINT16_MAX < tmp)
    return error_code_ = E_OUT_OF_RANGE;

  value = static_cast<uint16_t>(tmp);
  return error_code_ = E_NO_ERROR;
}

PropertyClient::error_t PropertyClient::get(const string& key, float& value, const rclcpp::Duration& timeout)
{
  return getProperty<GetDouble, kGetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const bool& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetBool, kSetBoolSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const int& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetInt, kSetIntSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const double& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetDouble, kSetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const string& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetString, kSetStringSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const uint8_t& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetInt, kSetIntSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const uint16_t& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetInt, kSetIntSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const float& value, const rclcpp::Duration& timeout)
{
  return setProperty<SetDouble, kSetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::save(const rclcpp::Duration& timeout)
{
  auto client = node_->create_client<Trigger>(path::join(ns_, kSaveFileSrv));
  if (!client->wait_for_service(timeout.to_chrono<chrono::nanoseconds>()))
    return error_code_ = E_FAILED_TO_CONNECT;

  const auto req = make_shared<Trigger::Request>();

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

PropertyClient::error_t PropertyClient::errorCode() const
{
  return error_code_;
}

string PropertyClient::errorMessage() const
{
  switch (error_code_)
  {
    case E_NO_ERROR:
      return "No error.";
    case E_FAILED_TO_CONNECT:
      return "Failed to connect to service server.";
    case E_FAILED_TO_CALL:
      return "Failed to call service.";
    case E_OUT_OF_RANGE:
      return "The value is out of numerical range.";
    case E_SERVER_ERROR:
      return server_error_msg_;
    default:
      throw;
  }
}
}  // namespace ptree
