// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/vehicle_configuration.hpp"

#include <memory>
#include <mutex>
#include <utility>

#include <QEventLoop>
#include <QTimer>
#include <rclcpp/qos.hpp>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>

namespace tobas
{
namespace gui
{
namespace gcs
{
std::expected<VehicleConfiguration, QString>
loadVehicleConfiguration(const rclcpp::Node::SharedPtr& node, const std::string& ns, int timeout_ms)
{
  struct ReceivedModels
  {
    std::mutex mutex;
    kdl::Tree::ConstSharedPtr tree;
    Drone::ConstSharedPtr drone;
  };

  // Shared ownership keeps executor callbacks safe even when a timed-out subscription is being removed.
  const auto received = std::make_shared<ReceivedModels>();
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  const auto tree_sub = node->create_subscription<kdl::Tree>(
    path::join(ns, kRemoteIfaceNS, topic::kKdlTree),
    qos,
    [received](const kdl::Tree::ConstSharedPtr& msg)
    {
      const std::lock_guard lock(received->mutex);
      received->tree = msg;
    });
  const auto drone_sub = node->create_subscription<Drone>(
    path::join(ns, kRemoteIfaceNS, topic::kDrone),
    qos,
    [received](const Drone::ConstSharedPtr& msg)
    {
      const std::lock_guard lock(received->mutex);
      received->drone = msg;
    });

  QEventLoop loop;
  QTimer poll;
  QTimer timeout;
  timeout.setSingleShot(true);
  QObject::connect(
    &poll,
    &QTimer::timeout,
    &loop,
    [&loop, &received]
    {
      const std::lock_guard lock(received->mutex);
      if (received->tree && received->drone) {
        loop.quit();
      }
    });
  QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
  poll.start(10);
  timeout.start(timeout_ms);
  loop.exec(QEventLoop::AllEvents | QEventLoop::ExcludeUserInputEvents);

  const std::lock_guard lock(received->mutex);
  if (!received->tree || !received->drone) {
    return std::unexpected("Timed out waiting for the vehicle configuration from " + QString::fromStdString(ns) + ".");
  }

  return VehicleConfiguration{ *received->tree, *received->drone };
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
