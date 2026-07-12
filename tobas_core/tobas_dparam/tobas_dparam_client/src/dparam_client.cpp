// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_dparam_client/dparam_client.hpp"

#include <tobas_dparam_common/constants.hpp>

namespace tobas
{
namespace dparam
{
DynamicParamClient::DynamicParamClient(rclcpp::Node::SharedPtr node, const std::string& node_name, const std::string& ns)
  : node_(node)
  , node_name_(node_name)
  , bool_sc_(node_, path::join(ns, kSetBoolSrv))
  , int_sc_(node_, path::join(ns, kSetIntSrv))
  , double_sc_(node_, path::join(ns, kSetDoubleSrv))
  , string_sc_(node_, path::join(ns, kSetStringSrv))
{
}

DynamicParamClient::Error DynamicParamClient::setBool(const std::string& param_name, const bool& value)
{
  return setParam(bool_sc_, param_name, value);
}

DynamicParamClient::Error DynamicParamClient::setInt(const std::string& param_name, const long& value)
{
  return setParam(int_sc_, param_name, value);
}

DynamicParamClient::Error DynamicParamClient::setDouble(const std::string& param_name, const long& value)
{
  return setParam(double_sc_, param_name, value);
}

DynamicParamClient::Error DynamicParamClient::setString(const std::string& param_name, const std::string& value)
{
  return setParam(string_sc_, param_name, value);
}

DynamicParamClient::Error DynamicParamClient::errorCode() const
{
  return error_code_;
}

const char* DynamicParamClient::errorMessage() const
{
  switch (error_code_) {
    case kNoError:
      return "";
    case kServiceNotReady:
      return "Dynamic parameter server is not ready.";
    case kServerError:
      return "Dynamic parameter service finished with error.";  // TODO: Get a specific error message from the server.
    default:
      throw;
  }
}
}  // namespace dparam
}  // namespace tobas
