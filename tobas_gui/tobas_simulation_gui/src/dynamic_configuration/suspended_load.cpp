// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/dynamic_configuration/suspended_load.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_eigen_conversions/eigen_msg.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_string_tools/core.hpp>

#include "tobas_simulation_gui/dynamic_configuration/constants.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace sim
{
SuspendedLoadWidget::SuspendedLoadWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  const auto title = new qt::Label("Suspended Load", cmn::kLabelPSize, QFont::Bold);

  attach_detach_btn_ = new qt::ToggleButton("Attach", "Detach");
  attach_detach_btn_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);

  attach_point_ = new qt::Vector3dEditVertical();
  attach_point_->setDecimals(3);
  attach_point_->setSuffix(" m");

  load_size_ = new qt::Vector3dEditVertical();
  load_size_->setDecimals(3);
  load_size_->setMinimum(1e-3);
  load_size_->setSuffix(" m");

  load_mass_ = new qt::DoubleSpinBox();
  load_mass_->setDecimals(3);
  load_mass_->setMinimum(1e-3);
  load_mass_->setSuffix(" kg");

  cable_length_ = new qt::DoubleSpinBox();
  cable_length_->setDecimals(1);
  cable_length_->setMinimum(0.1);
  cable_length_->setSuffix(" m");

  cable_young_ = new qt::SpinBox();
  cable_young_->setMinimum(1);
  cable_young_->setSuffix(" MPa");

  cable_csa_ = new qt::SpinBox();
  cable_csa_->setMinimum(1);
  cable_csa_->setSuffix(QString::fromStdString(str::convertToSuperscript(" mm^2")));

  setParamsToDefault();

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(title);
  header_cols->addStretch();
  header_cols->addWidget(attach_detach_btn_);

  const auto form = new qt::FormLayout();
  form->addVAlignedRow("Vehicle Attachment Point", attach_point_);
  form->addVAlignedRow("Load Size", load_size_);
  form->addVAlignedRow("Load Mass", load_mass_);
  form->addVAlignedRow("Cable Length", cable_length_);
  form->addVAlignedRow("Cable Young Modulus", cable_young_);
  form->addVAlignedRow("Cable Cross-Sectional Area", cable_csa_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_cols);
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(attach_detach_btn_, &qt::ToggleButton::checked, this, &self::onAttachRequested);
  connect(attach_detach_btn_, &qt::ToggleButton::unchecked, this, &self::onDetachRequested);
}

void SuspendedLoadWidget::updateNamespace(const std::string& ns)
{
  attach_sc_ =
    std::make_shared<ros2::SyncServiceClient<AttachSrv>>(node_, path::join(ns, gazebo::kAttachSuspenedLoadSrv));
  detach_sc_ =
    std::make_shared<ros2::SyncServiceClient<DetachSrv>>(node_, path::join(ns, gazebo::kDetachSuspenedLoadSrv));

  setParamsToDefault();
}

bool SuspendedLoadWidget::start(ch::milliseconds timeout)
{
  bool success = true;
  QString message;

  qt::startThreadAndWait(
    [&]()
    {
      if (!attach_sc_->waitForService(timeout)) {
        success = false;
        message = "Failed to connect to \"" + QString(gazebo::kAttachSuspenedLoadSrv) + "\" service server.";
        return;
      }
      if (!detach_sc_->waitForService(timeout)) {
        success = false;
        message = "Failed to connect to \"" + QString(gazebo::kDetachSuspenedLoadSrv) + "\" service server.";
        return;
      }
    });

  if (!success) {
    qt::qErrorBox(this, message);
    return false;
  }

  return true;
}

void SuspendedLoadWidget::reset()
{
  attach_detach_btn_->setChecked(false);
}

void SuspendedLoadWidget::setParamsToDefault()
{
  attach_point_->setVector(Eigen::Vector3d::Zero());
  load_size_->setVector(Eigen::Vector3d::Constant(kDefaultLoadSize));
  load_mass_->setValue(kDefaultLoadMass);
  cable_length_->setValue(kDefaultCableLength);
  cable_young_->setValue(kDefaultCableYoungModulus);
  cable_csa_->setValue(kDefaultCableCrossSectionArea);
}

void SuspendedLoadWidget::onAttachRequested()
{
  const auto req = std::make_shared<AttachSrv::Request>();
  tf::vectorEigenToMsg(attach_point_->vector(), req->attachment_point);
  req->load_sx = load_size_->x();
  req->load_sy = load_size_->y();
  req->load_sz = load_size_->z();
  req->load_mass = load_mass_->value();
  req->cable_length = cable_length_->value();
  req->cable_young_modulus = cable_young_->value() * 1e+6;
  req->cable_cross_sectional_area = cable_csa_->value() * 1e-6;

  if (!attach_sc_->call(req, kServiceCallTimeout)) {
    qt::qErrorBox(this, "Failed to call \"" + QString(gazebo::kAttachSuspenedLoadSrv) + "\" service.");
    reset();
    return;
  }

  const auto res = attach_sc_->getResponse();
  if (!res->success) {
    qt::qErrorBox(this, "Failed to attach a load: " + QString::fromStdString(res->message));
    reset();
    return;
  }
}

void SuspendedLoadWidget::onDetachRequested()
{
  const auto req = std::make_shared<DetachSrv::Request>();

  if (!detach_sc_->call(req, kServiceCallTimeout)) {
    qt::qErrorBox(this, "Failed to call \"" + QString(gazebo::kDetachSuspenedLoadSrv) + "\" service.");
    reset();
    return;
  }

  const auto res = detach_sc_->getResponse();
  if (!res->success) {
    qt::qErrorBox(this, "Failed to detach the load: " + QString::fromStdString(res->message));
    reset();
    return;
  }
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
