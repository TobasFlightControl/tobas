// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>

#include <tobas_dparam_common/constants.hpp>
#include <tobas_node/node.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <tobas_dparam_msgs/srv/set_bool.hpp>
#include <tobas_dparam_msgs/srv/set_double.hpp>
#include <tobas_dparam_msgs/srv/set_int.hpp>
#include <tobas_dparam_msgs/srv/set_string.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace tobas_dparam_msgs::srv;

namespace tobas::dparam
{
class DynamicParamServer : public BaseNode
{
  using super = BaseNode;

public:
  explicit DynamicParamServer(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::unordered_map<std::string, rclcpp::AsyncParametersClient::SharedPtr> clients_;

  template <typename SrvType, typename ValueType>
  void addService(const std::string& _service_name);

  template <typename SrvType, typename ValueType>
  void callback(
    const typename rclcpp::Service<SrvType>::SharedPtr& _server,
    const std::shared_ptr<rmw_request_id_t>& _header,
    const typename SrvType::Request::SharedPtr& _req);
};

DynamicParamServer::DynamicParamServer(const rclcpp::NodeOptions& _options)
  : super("dynamic_parameter_server", nodeOptions_Default(_options))
{
  addService<SetBool, bool>(kSetBoolSrv);
  addService<SetInt, long>(kSetIntSrv);
  addService<SetDouble, long>(kSetDoubleSrv);
  addService<SetString, std::string>(kSetStringSrv);
}

template <typename SrvType, typename ValueType>
void DynamicParamServer::addService(const std::string& _service_name)
{
  using Callback = std::function<void(
    typename rclcpp::Service<SrvType>::SharedPtr, std::shared_ptr<rmw_request_id_t>, typename SrvType::Request::SharedPtr)>;
  const Callback cb = std::bind(&DynamicParamServer::callback<SrvType, ValueType>, this, _1, _2, _3);
  services_.push_back(create_service<SrvType>(_service_name, cb, rclcpp::ServicesQoS()));
}

template <typename SrvType, typename ValueType>
void DynamicParamServer::callback(
  const typename rclcpp::Service<SrvType>::SharedPtr& _server,
  const std::shared_ptr<rmw_request_id_t>& _header,
  const typename SrvType::Request::SharedPtr& _req)
{
  const auto respond = [_server, _header](bool _success, const std::string& _message)
  {
    typename SrvType::Response response;
    response.success = _success;
    response.message = _message;
    _server->send_response(*_header, response);
  };

  if (_req->node_name.empty()) {
    respond(false, "The node name is empty.");
    return;
  }

  // Reuse a client per target node, caching new clients only after service discovery succeeds.
  auto client_it = clients_.find(_req->node_name);
  if (client_it == clients_.end()) {
    TOBAS_INFO("Creating a new parameter client for '", _req->node_name, "'.");
    const auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, _req->node_name);
    if (!client->wait_for_service(1s)) {
      respond(false, "Failed to find '" + _req->node_name + "'.");
      return;
    }
    client_it = clients_.insert({ _req->node_name, client }).first;
  }

  // A cached client may outlive its target node, so recheck service availability before sending.
  const auto& client = client_it->second;
  if (!client->service_is_ready()) {
    respond(false, "The parameter server is not ready");
    return;
  }

  // Return before the parameter response arrives so one worker can also process that response.
  const auto send_response = [respond](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> _future)
  {
    const auto results = _future.get();
    if (results.size() != 1) {
      respond(false, "Result size mismatch.");
      return;
    }
    const auto& result = results.front();
    respond(result.successful, result.successful ? "" : result.reason);
  };
  client->set_parameters({ rclcpp::Parameter(_req->param_name, _req->value) }, send_response);
}
}  // namespace tobas::dparam

int main(int _argc, char* _argv[])
{
  rclcpp::init(_argc, _argv);
  const auto node = std::make_shared<tobas::dparam::DynamicParamServer>();
  rclcpp::executors::EventsCBGExecutor exec(rclcpp::ExecutorOptions(), 1);
  exec.add_node(node);
  exec.spin();
}
