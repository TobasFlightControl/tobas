// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_property_client/property_client.hpp"

using namespace std;

namespace tobas
{
namespace ptree
{
PropertyClient::PropertyClient(rclcpp::Node::SharedPtr node, const string& section)
  : node_(node)
  , section_(section)
  , get_bool_sc_(node, kGetBoolSrv)
  , get_int_sc_(node, kGetIntSrv)
  , get_double_sc_(node, kGetDoubleSrv)
  , get_string_sc_(node, kGetStringSrv)
  , set_bool_sc_(node, kSetBoolSrv)
  , set_int_sc_(node, kSetIntSrv)
  , set_double_sc_(node, kSetDoubleSrv)
  , set_string_sc_(node, kSetStringSrv)
  , save_sc_(node, kSaveFileSrv)
{
}

PropertyClient::Error PropertyClient::get(const string& key, bool& value)
{
  return getProperty(get_bool_sc_, key, value);
}

PropertyClient::Error PropertyClient::get(const string& key, int& value)
{
  return getProperty(get_int_sc_, key, value);
}

PropertyClient::Error PropertyClient::get(const string& key, double& value)
{
  return getProperty(get_double_sc_, key, value);
}

PropertyClient::Error PropertyClient::get(const string& key, string& value)
{
  return getProperty(get_string_sc_, key, value);
}

PropertyClient::Error PropertyClient::get(const string& key, uint8_t& value)
{
  int tmp;

  if (getProperty(get_int_sc_, key, tmp) < 0) {
    return error_code_;
  }

  if (tmp < 0 || UINT8_MAX < tmp) {
    return error_code_ = kOutputRange;
  }

  value = static_cast<uint8_t>(tmp);
  return error_code_ = kNoError;
}

PropertyClient::Error PropertyClient::get(const string& key, uint16_t& value)
{
  int tmp;

  if (getProperty(get_int_sc_, key, tmp) < 0) {
    return error_code_;
  }

  if (tmp < 0 || UINT16_MAX < tmp) {
    return error_code_ = kOutputRange;
  }

  value = static_cast<uint16_t>(tmp);
  return error_code_ = kNoError;
}

PropertyClient::Error PropertyClient::get(const string& key, float& value)
{
  return getProperty(get_double_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const bool& value)
{
  return setProperty(set_bool_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const int& value)
{
  return setProperty(set_int_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const double& value)
{
  return setProperty(set_double_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const string& value)
{
  return setProperty(set_string_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const uint8_t& value)
{
  return setProperty(set_int_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const uint16_t& value)
{
  return setProperty(set_int_sc_, key, value);
}

PropertyClient::Error PropertyClient::set(const string& key, const float& value)
{
  return setProperty(set_double_sc_, key, value);
}

PropertyClient::Error PropertyClient::save()
{
  const auto req = make_shared<std_srvs::srv::Trigger::Request>();
  const auto res = save_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServiceNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

PropertyClient::Error PropertyClient::errorCode() const
{
  return error_code_;
}

const char* PropertyClient::errorMessage() const
{
  switch (error_code_) {
    case kNoError:
      return "";
    case kServiceNotReady:
      return "Property server is not ready.";
    case kOutputRange:
      return "The property value is out of numerical range.";
    case kServerError:
      return server_error_msg_.c_str();
    default:
      throw;
  }
}
}  // namespace ptree
}  // namespace tobas
