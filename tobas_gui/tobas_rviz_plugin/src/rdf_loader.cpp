// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/rdf_loader.hpp"

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.rdf_loader");
}
}  // namespace

RDFLoader::RDFLoader(
  const rclcpp::Node::SharedPtr& node,
  const std::string& ros_name,
  bool default_continuous_value,
  double default_timeout)
  : ros_name_(ros_name)
{
  const auto start = node->now();

  urdf_string_ = urdf_ssp_.loadInitialValue(
    node,
    ros_name,
    [this](const std::string& new_urdf_string) { return urdfUpdateCallback(new_urdf_string); },
    default_continuous_value,
    default_timeout);

  if (!loadFromStrings()) {
    return;
  }

  RCLCPP_INFO_STREAM(getLogger(), "Loaded robot model in " << (node->now() - start).seconds() << " seconds.");
}

const ::urdf::ModelInterfaceSharedPtr& RDFLoader::getURDF() const
{
  return urdf_;
}

void RDFLoader::setNewModelCallback(const NewModelCallback& cb)
{
  new_model_cb_ = cb;
}

bool RDFLoader::loadFromStrings()
{
  auto urdf = std::make_unique<::urdf::Model>();
  if (!urdf->initString(urdf_string_)) {
    RCLCPP_ERROR(getLogger(), "Failed to parse URDF.");
    return false;
  }

  urdf_ = std::move(urdf);
  return true;
}

void RDFLoader::urdfUpdateCallback(const std::string& new_urdf_string)
{
  urdf_string_ = new_urdf_string;
  if (!loadFromStrings()) {
    return;
  }
  if (new_model_cb_) {
    new_model_cb_();
  }
}
}  // namespace tobas
