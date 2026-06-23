// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/node.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_real_common/ros_interface.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>
#include <tobas_real_msgs/srv/set_imu_params.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

#include "./ros_interface.hpp"

namespace tobas
{
void RosInterfaceNode::registerServices()
{
  addService<tobas_msgs::srv::SetArm>(service::kSetArm);
  addService<tobas_msgs::srv::GetGnssOrigin>(service::kGetGnssOrigin);
  addService<tobas_msgs::srv::SetGnssOrigin>(service::kSetGnssOrigin);
  addService<tobas_msgs::srv::BagRecordStart>(service::kRosbagRecordStart);
  addService<tobas_msgs::srv::BagRecordStop>(service::kRosbagRecordStop);
  addService<std_srvs::srv::Trigger>(service::kRosbagClean);
  addService<tobas_msgs::srv::GetRotorControlGains>(service::kGetRotorControlGains);
  addService<tobas_msgs::srv::SetRotorControlGains>(service::kSetRotorControlGains);
  addService<std_srvs::srv::Trigger>(service::kSaveRotorControlGains);
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kImuFilterConfigServer, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kObserver, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kController, service::kGetDynamicParams));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(node::kRcTeleop, service::kGetDynamicParams));
  addService<tobas_real_msgs::srv::SetImuParams>(real::handler::imu::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetMagnetometerParams>(real::handler::mag::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetRcInputParams>(real::handler::rcin::kSetParamSrv);
}
}  // namespace tobas
