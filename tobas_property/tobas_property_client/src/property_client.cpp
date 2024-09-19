#include <std_srvs/srv/trigger.hpp>

#include <tobas_property_msgs/srv/get_bool.hpp>
#include <tobas_property_msgs/srv/get_int.hpp>
#include <tobas_property_msgs/srv/get_double.hpp>
#include <tobas_property_msgs/srv/get_string.hpp>
#include <tobas_property_msgs/srv/set_bool.hpp>
#include <tobas_property_msgs/srv/set_int.hpp>
#include <tobas_property_msgs/srv/set_double.hpp>
#include <tobas_property_msgs/srv/set_string.hpp>

#include "../include/tobas_property_client/property_client.hpp"

using namespace std;
using namespace std_srvs::srv;
using namespace tobas_property_msgs::srv;

namespace ptree
{
PropertyClient::PropertyClient(rclcpp::Node::SharedPtr node, const string& ns, const string& section)
  : node_(node), ns_(ns), section_(section)
{
}

PropertyClient::error_t PropertyClient::get(const string& key, bool& value)
{
  return getProperty<GetBool, kGetBoolSrv>(key, value);
}

PropertyClient::error_t PropertyClient::get(const string& key, int& value)
{
  return getProperty<GetInt, kGetIntSrv>(key, value);
}

PropertyClient::error_t PropertyClient::get(const string& key, double& value)
{
  return getProperty<GetDouble, kGetDoubleSrv>(key, value);
}

PropertyClient::error_t PropertyClient::get(const string& key, string& value)
{
  return getProperty<GetString, kGetStringSrv>(key, value);
}

PropertyClient::error_t PropertyClient::get(const string& key, uint8_t& value)
{
  int tmp;

  if (getProperty<GetInt, kGetIntSrv>(key, tmp) < 0)
    return error_code_;

  if (tmp < 0 || UINT8_MAX < tmp)
    return error_code_ = E_OUT_OF_RANGE;

  value = static_cast<uint8_t>(tmp);
  return error_code_ = E_NO_ERROR;
}

PropertyClient::error_t PropertyClient::get(const string& key, uint16_t& value)
{
  int tmp;

  if (getProperty<GetInt, kGetIntSrv>(key, tmp) < 0)
    return error_code_;

  if (tmp < 0 || UINT16_MAX < tmp)
    return error_code_ = E_OUT_OF_RANGE;

  value = static_cast<uint16_t>(tmp);
  return error_code_ = E_NO_ERROR;
}

PropertyClient::error_t PropertyClient::get(const string& key, float& value)
{
  return getProperty<GetDouble, kGetDoubleSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const bool& value)
{
  return setProperty<SetBool, kSetBoolSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const int& value)
{
  return setProperty<SetInt, kSetIntSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const double& value)
{
  return setProperty<SetDouble, kSetDoubleSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const string& value)
{
  return setProperty<SetString, kSetStringSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const uint8_t& value)
{
  return setProperty<SetInt, kSetIntSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const uint16_t& value)
{
  return setProperty<SetInt, kSetIntSrv>(key, value);
}

PropertyClient::error_t PropertyClient::set(const string& key, const float& value)
{
  return setProperty<SetDouble, kSetDoubleSrv>(key, value);
}

PropertyClient::error_t PropertyClient::save()
{
  const auto client = node_->create_client<Trigger>(path::join(ns_, kSaveFileSrv));
  if (!client->service_is_ready())
    return error_code_ = E_SERVICE_NOT_READY;

  const auto req = make_shared<Trigger::Request>();

  auto future = client->async_send_request(req);
  future.wait();

  const auto res = future.get();
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

const char* PropertyClient::errorMessage() const
{
  switch (error_code_)
  {
    case E_NO_ERROR:
      return "";
    case E_SERVICE_NOT_READY:
      return "Property server is not ready.";
    case E_OUT_OF_RANGE:
      return "The property value is out of numerical range.";
    case E_SERVER_ERROR:
      return server_error_msg_.c_str();
    default:
      throw;
  }
}
}  // namespace ptree
