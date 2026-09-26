// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/dynamic_configuration/fixed_load.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <eigen3/Eigen/Geometry>

#include <tobas_eigen_conversions/eigen_msg.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_simulation_gui/dynamic_configuration/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
FixedLoadWidget::FixedLoadWidget()
{
  const auto title = new qt::Label("Fixed Load", cmn::kLabelPSize, QFont::Bold);

  attach_detach_btn_ = new qt::ToggleButton("Attach", "Detach");
  attach_detach_btn_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);

  load_position_ = new qt::Vector3dEditVertical({ "X", "Y", "Z" });
  load_position_->setDecimals(3);
  load_position_->setSuffix(" m");

  load_angle_ = new qt::Vector3dEditVertical({ "Roll", "Pitch", "Yaw" });
  load_angle_->setDecimals(1);
  load_angle_->setRange(-180.0, 180.0);
  load_angle_->setSuffix(" deg");

  load_size_ = new qt::Vector3dEditVertical();
  load_size_->setDecimals(3);
  load_size_->setMinimum(1e-3);
  load_size_->setSuffix(" m");

  load_mass_ = new qt::DoubleSpinBox();
  load_mass_->setDecimals(3);
  load_mass_->setMinimum(1e-3);
  load_mass_->setSuffix(" kg");

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(title);
  header_cols->addStretch();
  header_cols->addWidget(attach_detach_btn_);

  const auto pose_cols = new QHBoxLayout();
  pose_cols->addWidget(load_position_, 1);
  pose_cols->addWidget(load_angle_, 1);

  const auto form = new qt::FormLayout();
  form->addVAlignedRow("Load Pose", pose_cols);
  form->addVAlignedRow("Load Size", load_size_);
  form->addVAlignedRow("Load Mass", load_mass_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_cols);
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(attach_detach_btn_, &qt::ToggleButton::checked, this, &self::onAttachButtonClicked);
  connect(attach_detach_btn_, &qt::ToggleButton::unchecked, this, &self::onDetachButtonClicked);

  reset();
}

void FixedLoadWidget::reset()
{
  attach_detach_btn_->setChecked(false);

  setParamsToDefault();
}

void FixedLoadWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  attach_sc_.emplace(node, path::join(ns, gazebo::kAttachFixedLoadSrv));
  detach_sc_.emplace(node, path::join(ns, gazebo::kDetachFixedLoadSrv));
}

void FixedLoadWidget::clearRosInterfaces()
{
  attach_sc_.reset();
  detach_sc_.reset();
}

void FixedLoadWidget::setParamsToDefault()
{
  load_position_->setVector({ 0.0, 0.0, -0.5 });
  load_angle_->setVector(Eigen::Vector3d::Zero());
  load_size_->setVector(Eigen::Vector3d::Constant(0.3));
  load_mass_->setValue(1.0);
}

void FixedLoadWidget::onAttachButtonClicked()
{
  const auto req = std::make_shared<AttachSrv::Request>();

  tf::pointEigenToMsg(load_position_->vector(), req->load_pose.position);

  const auto roll = st::deg2rad(load_angle_->x());
  const auto pitch = st::deg2rad(load_angle_->y());
  const auto yaw = st::deg2rad(load_angle_->z());
  const auto orientation = kdl::Quaternion::RPY(roll, pitch, yaw);
  kdl::quaternionKDLToMsg(orientation, req->load_pose.orientation);

  tf::vectorEigenToMsg(load_size_->vector(), req->load_size);

  req->load_mass = load_mass_->value();

  const auto res = attach_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to call '" + QString(gazebo::kAttachFixedLoadSrv) + "' service.");
    attach_detach_btn_->setChecked(false);
    return;
  }

  if (!res->success) {
    qt::qErrorBox(this, "Failed to attach a fixed load: " + QString::fromStdString(res->message));
    attach_detach_btn_->setChecked(false);
    return;
  }
}

void FixedLoadWidget::onDetachButtonClicked()
{
  const auto req = std::make_shared<DetachSrv::Request>();

  const auto res = detach_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to call '" + QString(gazebo::kDetachFixedLoadSrv) + "' service.");
    attach_detach_btn_->setChecked(true);
    return;
  }

  if (!res->success) {
    qt::qErrorBox(this, "Failed to detach the fixed load: " + QString::fromStdString(res->message));
    attach_detach_btn_->setChecked(true);
    return;
  }
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
