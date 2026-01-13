#include "tobas_simulation_gui/dynamic_configuration/suspended_load.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/dynamic_configuration/constants.hpp"

namespace gui
{
namespace sim
{
SuspendedLoadWidget::SuspendedLoadWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  const auto title = new qt::Label("Suspended Load", cmn::kLabelPSize, QFont::Bold);

  attach_detach_btn_ = new qt::ToggleButton("Attach", "Detach");
  attach_detach_btn_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);

  attach_point_ = new qt::Vector3dEdit();
  attach_point_->setDecimals(3);
  attach_point_->setSuffix(" m");

  load_size_ = new qt::Vector3dEdit();
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
  cable_csa_->setSuffix(" mm^2");

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

bool SuspendedLoadWidget::start()
{
  if (!attach_sc_->waitForService()) {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(gazebo::kAttachSuspenedLoadSrv) + "\" service server.");
    return false;
  }
  if (!detach_sc_->waitForService()) {
    qt::qErrorBox(this, "Failed to connect to \"" + QString(gazebo::kDetachSuspenedLoadSrv) + "\" service server.");
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
  // TODO
}

void SuspendedLoadWidget::onDetachRequested()
{
  // TODO
}
}  // namespace sim
}  // namespace gui
