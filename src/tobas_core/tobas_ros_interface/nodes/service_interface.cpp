// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/client.hpp>
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>

#include <tobas_constants/ros.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/util.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>
#include <tobas_msgs/srv/attach_load.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_msgs/srv/detach_load.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_real_msgs/srv/set_imu_params.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

namespace tobas
{
class ServiceInterfaceNode : public BaseNode
{
  using super = BaseNode;

public:
  explicit ServiceInterfaceNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  template <typename SrvType>
  void addService(const std::string& _service_name);
};

ServiceInterfaceNode::ServiceInterfaceNode(const rclcpp::NodeOptions& _options)
  : super("service_interface", nodeOptions_Default(_options))
{
  addService<tobas_msgs::srv::SetArm>(service::kSetArm);
  addService<tobas_msgs::srv::GetGnssOrigin>(service::kGetGnssOrigin);
  addService<tobas_msgs::srv::SetGnssOrigin>(service::kSetGnssOrigin);
  addService<tobas_msgs::srv::BagRecordStart>(service::kRosbagRecordStart);
  addService<tobas_msgs::srv::BagRecordStop>(service::kRosbagRecordStop);
  addService<std_srvs::srv::Trigger>(service::kRosbagClean);
  addService<tobas_msgs::srv::AttachLoad>(service::kAttachLoad);
  addService<tobas_msgs::srv::DetachLoad>(service::kDetachLoad);

  addService<tobas_real_msgs::srv::SetImuParams>(real::handler::imu::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetMagnetometerParams>(real::handler::mag::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetRcInputParams>(real::handler::rcin::kSetParamSrv);

  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kImuFilterConfigServer, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kRpmControlConfigServer, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kObserver, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kController, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kRcTeleop, service::kGetDynamicParams));
}

template <typename SrvType>
void ServiceInterfaceNode::addService(const std::string& _service_name)
{
  const auto client = create_client<SrvType>(_service_name);

  // The server supplies its own handle, so the callback only needs to retain the client.
  const auto cb = [this, client](
                    typename rclcpp::Service<SrvType>::SharedPtr _server,
                    std::shared_ptr<rmw_request_id_t> _header,
                    typename SrvType::Request::SharedPtr _req)
  {
    if (!client->service_is_ready()) {
      TOBAS_ERROR("'", client->get_service_name(), "' service is not ready.");
      typename SrvType::Response response;
      _server->send_response(*_header, response);
      return;
    }
    // Defer the response so no executor worker is blocked while the downstream service runs.
    const auto send_response = [_server, _header](typename rclcpp::Client<SrvType>::SharedFuture _future)
    {
      const auto response = _future.get();
      _server->send_response(*_header, *response);
    };
    client->async_send_request(_req, send_response);
  };

  services_.push_back(create_service<SrvType>(addIfaceNS(_service_name), cb, rclcpp::ServicesQoS()));
}
}  // namespace tobas

int main(int _argc, char* _argv[])
{
  rclcpp::init(_argc, _argv);
  const auto node = std::make_shared<tobas::ServiceInterfaceNode>();
  rclcpp::executors::EventsCBGExecutor exec(rclcpp::ExecutorOptions(), 1);
  exec.add_node(node);
  exec.spin();
}
