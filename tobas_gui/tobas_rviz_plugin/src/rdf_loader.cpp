// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/rdf_loader.hpp"

#include <QDebug>

namespace tobas::rviz
{
RDFLoader::RDFLoader(
  const rclcpp::Node::SharedPtr& _node,
  const std::string& _ros_name,
  bool _default_continuous_value,
  double _default_timeout)
  : ros_name_(_ros_name)
{
  const auto start = _node->now();

  urdf_string_ = urdf_ssp_.loadInitialValue(
    _node,
    _ros_name,
    [this](const std::string& _new_urdf_string) { return urdfUpdateCallback(_new_urdf_string); },
    _default_continuous_value,
    _default_timeout);

  if (!loadFromStrings()) {
    return;
  }

  qInfo() << "Loaded robot model in" << (_node->now() - start).seconds() << "seconds.";
}

const urdf::ModelInterfaceSharedPtr& RDFLoader::getURDF() const
{
  return urdf_;
}

void RDFLoader::setNewModelCallback(const NewModelCallback& _cb)
{
  new_model_cb_ = _cb;
}

bool RDFLoader::loadFromStrings()
{
  auto urdf = std::make_unique<urdf::Model>();
  if (!urdf->initString(urdf_string_)) {
    qCritical() << "Failed to parse URDF.";
    return false;
  }

  urdf_ = std::move(urdf);
  return true;
}

void RDFLoader::urdfUpdateCallback(const std::string& _new_urdf_string)
{
  urdf_string_ = _new_urdf_string;
  if (!loadFromStrings()) {
    return;
  }
  if (new_model_cb_) {
    new_model_cb_();
  }
}
}  // namespace tobas::rviz
