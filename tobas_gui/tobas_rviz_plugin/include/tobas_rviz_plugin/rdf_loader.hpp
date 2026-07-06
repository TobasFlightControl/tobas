// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <urdf/model.h>
#include <rclcpp/rclcpp.hpp>

#include "./synchronized_string_parameter.hpp"

namespace tobas
{
class RDFLoader
{
  using NewModelCallback = std::function<void()>;

public:
  explicit RDFLoader(
    const rclcpp::Node::SharedPtr& node,
    const std::string& ros_name = "robot_description",
    bool default_continuous_value = false,
    double default_timeout = 10.0);

  const urdf::ModelInterfaceSharedPtr& getURDF() const;
  void setNewModelCallback(const NewModelCallback& cb);

private:
  bool loadFromStrings();
  void urdfUpdateCallback(const std::string& new_urdf_string);

  std::string ros_name_;
  std::string urdf_string_;
  SynchronizedStringParameter urdf_ssp_;
  urdf::ModelInterfaceSharedPtr urdf_;
  NewModelCallback new_model_cb_;
};
}  // namespace tobas
