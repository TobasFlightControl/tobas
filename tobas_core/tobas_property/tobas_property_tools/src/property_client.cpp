#include <std_srvs/Trigger.h>

#include <tobas_property_msgs/GetBool.h>
#include <tobas_property_msgs/GetInt.h>
#include <tobas_property_msgs/GetDouble.h>
#include <tobas_property_msgs/GetString.h>
#include <tobas_property_msgs/SetBool.h>
#include <tobas_property_msgs/SetInt.h>
#include <tobas_property_msgs/SetDouble.h>
#include <tobas_property_msgs/SetString.h>

#include "../include/tobas_property_tools/property_client.hpp"

using namespace std;

namespace ptree
{
PropertyClient::PropertyClient(ros::NodeHandle& nh, const string& ns, const string& section)
  : nh_(nh), ns_(ns), section_(section)
{
}

PropertyClient::error_t PropertyClient::get(const string& key, bool& value, const ros::Duration& timeout)
{
  return getProperty<tobas_property_msgs::GetBool, kGetBoolSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, int& value, const ros::Duration& timeout)
{
  return getProperty<tobas_property_msgs::GetInt, kGetIntSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, float& value, const ros::Duration& timeout)
{
  return getProperty<tobas_property_msgs::GetDouble, kGetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, double& value, const ros::Duration& timeout)
{
  return getProperty<tobas_property_msgs::GetDouble, kGetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::get(const string& key, string& value, const ros::Duration& timeout)
{
  return getProperty<tobas_property_msgs::GetString, kGetStringSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const bool& value, const ros::Duration& timeout)
{
  return setProperty<tobas_property_msgs::SetBool, kSetBoolSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const int& value, const ros::Duration& timeout)
{
  return setProperty<tobas_property_msgs::SetInt, kSetIntSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const float& value, const ros::Duration& timeout)
{
  return setProperty<tobas_property_msgs::SetDouble, kSetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const double& value, const ros::Duration& timeout)
{
  return setProperty<tobas_property_msgs::SetDouble, kSetDoubleSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::set(const string& key, const string& value, const ros::Duration& timeout)
{
  return setProperty<tobas_property_msgs::SetString, kSetStringSrv>(key, value, timeout);
}

PropertyClient::error_t PropertyClient::save(const ros::Duration& timeout)
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(path::join(ns_, kSaveFileSrv));
  if (!client.waitForExistence(timeout))
    return error_code_ = E_FAILED_TO_CONNECT;

  std_srvs::Trigger msg;
  if (!client.call(msg))
    return error_code_ = E_FAILED_TO_CALL;

  if (!msg.response.success)
  {
    server_error_msg_ = msg.response.message;
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
    case E_SERVER_ERROR:
      return server_error_msg_;
    default:
      throw;
  }
}
}  // namespace ptree
