// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_dparam_client/dparam_client.hpp"

#include <tobas_dparam_common/constants.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas
{
namespace dparam
{
DynamicParamClient::DynamicParamClient(
  rclcpp::Node::SharedPtr _node,
  const std::string& _node_name,
  const std::string& _ns)
  : node_(_node)
  , node_name_(_node_name)
  , bool_sc_(node_, path::join(_ns, kSetBoolSrv))
  , int_sc_(node_, path::join(_ns, kSetIntSrv))
  , double_sc_(node_, path::join(_ns, kSetDoubleSrv))
  , string_sc_(node_, path::join(_ns, kSetStringSrv))
{
}

DynamicParamClient::Error DynamicParamClient::setBool(const std::string& _param_name, bool _value)
{
  return setParam(bool_sc_, _param_name, _value);
}

DynamicParamClient::Error DynamicParamClient::setInt(const std::string& _param_name, long _value)
{
  return setParam(int_sc_, _param_name, _value);
}

DynamicParamClient::Error DynamicParamClient::setDouble(const std::string& _param_name, long _value)
{
  return setParam(double_sc_, _param_name, _value);
}

DynamicParamClient::Error DynamicParamClient::setString(const std::string& _param_name, const std::string& _value)
{
  return setParam(string_sc_, _param_name, _value);
}

DynamicParamClient::Error DynamicParamClient::errorCode() const
{
  return error_code_;
}

const char* DynamicParamClient::errorMessage() const
{
  switch (error_code_) {
    case kNoError:
      return "No error.";
    case kServiceNotReady:
      return "Dynamic parameter server is not ready.";
    case kServerError:
      return server_error_msg_.c_str();
    default:
      return "Unexpected error.";
  }
}
}  // namespace dparam
}  // namespace tobas
