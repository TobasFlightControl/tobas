// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_dparam_common/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>

#include <tobas_dparam_msgs/srv/set_bool.hpp>
#include <tobas_dparam_msgs/srv/set_double.hpp>
#include <tobas_dparam_msgs/srv/set_int.hpp>
#include <tobas_dparam_msgs/srv/set_string.hpp>

using namespace std::chrono_literals;
using namespace tobas_dparam_msgs::srv;

namespace tobas
{
namespace dparam
{
class DynamicParamServer : public BaseNode
{
  using self = DynamicParamServer;
  using super = BaseNode;

public:
  explicit DynamicParamServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::map<std::string, ros2::SyncParamClient> clients_;

  template <typename SrvType, typename ValueType>
  void callback(const typename SrvType::Request::ConstSharedPtr& req, const typename SrvType::Response::SharedPtr& res);
};

DynamicParamServer::DynamicParamServer(const rclcpp::NodeOptions& options)
  : super("dynamic_parameter_server", nodeOptions_Default(options))
{
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  services_.push_back(createService<SetBool>(kSetBoolSrv, &self::callback<SetBool, bool>, this, cb_group_));
  services_.push_back(createService<SetInt>(kSetIntSrv, &self::callback<SetInt, long>, this, cb_group_));
  services_.push_back(createService<SetDouble>(kSetDoubleSrv, &self::callback<SetDouble, long>, this, cb_group_));
  services_.push_back(createService<SetString>(kSetStringSrv, &self::callback<SetString, std::string>, this, cb_group_));
}

template <typename SrvType, typename ValueType>
void DynamicParamServer::callback(
  const typename SrvType::Request::ConstSharedPtr& req,
  const typename SrvType::Response::SharedPtr& res)
{
  if (req->node_name.empty()) {
    res->success = false;
    res->message = "The node name is empty.";
    return;
  }

  auto client_it = clients_.find(req->node_name);
  if (client_it == clients_.end()) {
    TOBAS_INFO("Creating a new parameter client for \"", req->node_name, "\".");
    ros2::SyncParamClient client(shared_from_this(), req->node_name);
    if (!client.waitForService(1s)) {
      res->success = false;
      res->message = "Failed to find \"" + req->node_name + "\".";
      return;
    }
    client_it = clients_.insert({ req->node_name, client }).first;
  }

  auto& client = client_it->second;
  if (client.template setParam<ValueType>(req->param_name, req->value) != ros2::SyncParamClient::kNoError) {
    res->success = false;
    res->message = client.errorMessage();
    return;
  }

  res->success = true;
  res->message.clear();
}
}  // namespace dparam
}  // namespace tobas

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<tobas::dparam::DynamicParamServer>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
}
