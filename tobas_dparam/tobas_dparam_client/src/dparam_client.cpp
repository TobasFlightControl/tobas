#include <tobas_dparam_common/constants.hpp>
#include <tobas_dparam_msgs/srv/set_bool.hpp>
#include <tobas_dparam_msgs/srv/set_int.hpp>
#include <tobas_dparam_msgs/srv/set_double.hpp>
#include <tobas_dparam_msgs/srv/set_string.hpp>

#include "../include/tobas_dparam_client/dparam_client.hpp"

using namespace std;
using namespace tobas_dparam_msgs::srv;

namespace dparam
{
DynamicParamClient::DynamicParamClient(rclcpp::Node::SharedPtr node, const string& node_name, const std::string& ns)
  : node_(node), node_name_(node_name), ns_(ns)
{
}

DynamicParamClient::error_t DynamicParamClient::set(const string& param_name, const bool& value)
{
  return setParam<SetBool, kSetBoolSrv>(param_name, value);
}

DynamicParamClient::error_t DynamicParamClient::set(const string& param_name, const int& value)
{
  return setParam<SetInt, kSetIntSrv>(param_name, value);
}

DynamicParamClient::error_t DynamicParamClient::set(const string& param_name, const double& value)
{
  return setParam<SetDouble, kSetDoubleSrv>(param_name, value);
}

DynamicParamClient::error_t DynamicParamClient::set(const string& param_name, const string& value)
{
  return setParam<SetString, kSetStringSrv>(param_name, value);
}

DynamicParamClient::error_t DynamicParamClient::errorCode() const
{
  return error_code_;
}

const char* DynamicParamClient::errorMessage() const
{
  switch (error_code_) {
    case E_NO_ERROR:
      return "";
    case E_SERVICE_NOT_READY:
      return "Dynamic parameter server is not ready.";
    case E_SERVER_ERROR:
      return "Dynamic parameter service finished with error.";  // TODO: サーバから具体的なエラーメッセージを得る
    default:
      throw;
  }
}
}  // namespace dparam
