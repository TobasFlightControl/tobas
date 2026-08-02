// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/dynamics.hpp"

#include <eigen3/Eigen/SVD>

#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineDynamicsWidget::EngineDynamicsWidget()
{
  data_ = new ParamGetterWidget_DoubleTable(
    "Engine Torque Test Data", "Select Test Data", { "Throttle [%]", "RPM", "Torque [Nm]" });
  data_->setDecimals({ 2, 0, 6 });
  data_->setMinimum({ 1e-2, 1, 1e-6 });
  data_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  data_->table()->setColumnsWidth(kDataTableColWidth);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(data_);
  setLayout(rows);
}

void EngineDynamicsWidget::setToDefaults()
{
  data_->table()->removeAll();
}

bool EngineDynamicsWidget::isValid()
{
  if (data_->count() == 0) {
    qt::qWarnBox(this, "Engine dynamics data is blank.");
    return false;
  }

  return true;
}

YAML::Node EngineDynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[data_->name()] = data_->getValue();

  return node;
}

void EngineDynamicsWidget::load(const YAML::Node& node)
{
  data_->setValue(node[data_->name()].as<Eigen::MatrixXd>());
}

std::pair<double, double> EngineDynamicsWidget::engineConstant() const
{
  const auto num_data = data_->count();
  const auto data_mat = data_->getValue();

  // Extract the data.
  const auto throttles = data_mat.col(0) / 100.0;       // [-]
  const auto speeds = data_mat.col(1) * st::kRpmToRps;  // [rad/s]
  const auto torques = data_mat.col(2);                 // [Nm]

  // Find engine dynamics constants by linear regression (memo: 3-28).
  const auto phi = M_PI_2 * throttles;  // [rad]
  const auto A_sqr_coefs = -(torques.array() / (1 - phi.array().cos())).square().matrix();
  const auto B_coefs = Eigen::VectorXd::Ones(num_data);

  Eigen::MatrixX2d Left(num_data, 2);
  Left.col(0) = A_sqr_coefs;
  Left.col(1) = B_coefs;

  const auto right = (torques.array() / speeds.array()).matrix();

  const auto sol = Left.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(right).eval();
  const auto A = std::sqrt(sol(0));
  const auto B = sol(1);

  return { A, B };
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
