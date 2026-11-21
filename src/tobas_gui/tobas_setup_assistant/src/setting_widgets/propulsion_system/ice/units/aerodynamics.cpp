// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/aerodynamics.hpp"

#include <QLabel>
#include <QVBoxLayout>

#include <eigen3/Eigen/SVD>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/constants.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/blade_theory.hpp"

namespace tobas::gui::sa::propulsion::ice
{
namespace
{
constexpr char kDataLabel[] = "Single-Propeller Test Data";
}  // namespace

AerodynamicsWidget::AerodynamicsWidget(const PropellerWidget* _propeller) : propeller_(_propeller)
{
  setWidgetResizable(false);

  const auto title = new QLabel(kDataLabel);
  title->setFont(qt::makeFont(cmn::kBodyPointSize, QFont::Bold));

  data_ = new DoubleMatrixEdit(kDataLabel, "Select Test Data", { "RPM", "Pitch [deg]", "Thrust [N]", "Torque [Nm]" });
  data_->setToolTip(
    "Single-propeller test measurements: "
    "rotational speed in rpm, variable-pitch angle in degrees, thrust in newtons, and torque in newton-meters. "
    "Use the same pitch zero position as in the Propeller tab.");
  data_->setDecimals({ 0, 2, 6, 6 });
  data_->setMinimum({ 1, -90, 1e-6, 1e-6 });
  data_->setMaximum({ INFINITY, 90, INFINITY, INFINITY });
  data_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

  // Layout
  const auto rows = new QVBoxLayout(this);
  rows->addWidget(title, 0);
  rows->addWidget(data_, 1);
}

void AerodynamicsWidget::setToDefaults()
{
  data_->clearData();
}

const char* AerodynamicsWidget::name() const
{
  return "Aerodynamics";
}

bool AerodynamicsWidget::isValid()
{
  if (data_->count() == 0) {
    qt::qWarnBox(this, "Propeller test data is blank.");
    return false;
  }

  if (const auto result = motorConst().validate(); !result) {
    qt::qWarnBox(
      this,
      "Failed to estimate thrust-model coefficients for the variable-pitch propeller:\n\n" +
        QString::fromStdString(result.error()));
    return false;
  }

  if (const auto result = momentConst().validate(); !result) {
    qt::qWarnBox(
      this,
      "Failed to estimate the moment constant of the variable pitch propeller:\n\n" +
        QString::fromStdString(result.error()));
    return false;
  }

  if (const auto result = dragConst().validate(); !result) {
    qt::qWarnBox(
      this,
      "Failed to estimate the drag constant of the variable pitch propeller:\n\n" +
        QString::fromStdString(result.error()));
    return false;
  }

  return true;
}

void AerodynamicsWidget::copyFrom(const BaseSelectedLinkSettingWidget* _src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget>(_src);
  data_->setData(derived->data_->getData());
}

YAML::Node AerodynamicsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kDataLabel] = data_->getData();

  return node;
}

void AerodynamicsWidget::load(const YAML::Node& _node)
{
  data_->setData(_node[kDataLabel].as<Eigen::MatrixXd>());
}

VppMotorConstant AerodynamicsWidget::motorConst() const
{
  const auto [speed, pitch, thrust, _] = getData();
  const auto num_data = speed.size();

  Eigen::MatrixX2d Left(num_data, 2);
  const auto speed_sqr = speed.cwiseAbs2().eval();
  Left.col(0) = speed_sqr;
  Left.col(1) = pitch.cwiseProduct(speed_sqr);

  const auto sol = Left.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(thrust).eval();
  const auto c0 = sol(0);
  const auto c1 = sol(1);

  return VppMotorConstant(c0, c1);
}

VppMomentConstant AerodynamicsWidget::momentConst() const
{
  const auto [c0, c1] = motorConst();
  const auto phi0 = -c0 / c1;

  const auto [_, pitch, thrust, torque] = getData();
  const auto num_data = pitch.size();

  Eigen::MatrixX3d Left(num_data, 3);
  const auto phi = (pitch.array() - phi0).matrix().eval();
  Left.col(0) = phi;
  Left.col(1).setOnes();
  Left.col(2) = phi.cwiseInverse();

  const auto cm = torque.cwiseProduct(thrust.cwiseInverse());

  const auto sol = Left.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(cm).eval();
  const auto a = sol(0);
  const auto b = sol(1);
  const auto c = sol(2);

  return VppMomentConstant(a, b, c, phi0);
}

VppDragConstant AerodynamicsWidget::dragConst() const
{
  const BladeTheory blade(
    propeller_->numBlades(), propeller_->radius(), propeller_->meanChord(), propeller_->pitchAngleNeutoral());
  return blade.dragConst();
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> AerodynamicsWidget::getData() const
{
  std::vector<double> speeds, pitches, thrusts, torques;

  const auto num_data = data_->count();
  const auto data_mat = data_->getData();

  const auto pitch_limit = propeller_->pitchAngleLimit();

  for (int i = 0; i < num_data; ++i) {
    const auto speed = st::rpm2rps(data_mat(i, 0));  // [rad/s]
    const auto pitch = st::deg2rad(data_mat(i, 1));  // [rad]
    const auto thrust = data_mat(i, 2);              // [N]
    const auto torque = data_mat(i, 3);              // [Nm]

    if (pitch_limit.inRange(pitch, 1e-3)) {
      speeds.push_back(speed);
      pitches.push_back(pitch);
      thrusts.push_back(thrust);
      torques.push_back(torque);
    }
  }

  return { eigen::fromStdVector(speeds),
           eigen::fromStdVector(pitches),
           eigen::fromStdVector(thrusts),
           eigen::fromStdVector(torques) };
}
}  // namespace tobas::gui::sa::propulsion::ice
