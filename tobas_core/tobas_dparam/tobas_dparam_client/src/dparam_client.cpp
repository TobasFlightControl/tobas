#include "tobas_dparam_client/dparam_client.hpp"

#include <tobas_dparam_common/constants.hpp>

#include <tobas_dparam_msgs/srv/set_bool.hpp>
#include <tobas_dparam_msgs/srv/set_double.hpp>
#include <tobas_dparam_msgs/srv/set_int.hpp>
#include <tobas_dparam_msgs/srv/set_string.hpp>

using namespace tobas_dparam_msgs::srv;

namespace tobas
{
namespace dparam
{
DynamicParamClient::DynamicParamClient(rclcpp::Node::SharedPtr node, const std::string& node_name, const std::string& ns)
  : node_(node), node_name_(node_name), ns_(ns)
{
}

DynamicParamClient::Error DynamicParamClient::setBool(const std::string& param_name, const bool& value)
{
  return setParam<SetBool, kSetBoolSrv>(param_name, value);
}

DynamicParamClient::Error DynamicParamClient::setInt(const std::string& param_name, const long& value)
{
  return setParam<SetInt, kSetIntSrv>(param_name, value);
}

DynamicParamClient::Error DynamicParamClient::setDouble(const std::string& param_name, const long& value)
{
  return setParam<SetDouble, kSetDoubleSrv>(param_name, value);
}

DynamicParamClient::Error DynamicParamClient::setString(const std::string& param_name, const std::string& value)
{
  return setParam<SetString, kSetStringSrv>(param_name, value);
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
      return "Dynamic parameter service finished with error.";  // TODO: サーバから具体的なエラーメッセージを得る
    default:
      throw;
  }
}
}  // namespace dparam
}  // namespace tobas
