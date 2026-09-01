// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/mission_executor/multicopter.hpp"

#include <QDebug>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace mission
{
namespace
{
constexpr char kWaypointMaxHorizontalVelocityParam[] = "waypoint/max_horizontal_velocity";
constexpr char kWaypointMaxHorizontalAccelParam[] = "waypoint/max_horizontal_accel";
constexpr char kWaypointMaxHorizontalJerkParam[] = "waypoint/max_horizontal_jerk";
constexpr char kWaypointMaxVerticalVelocityParam[] = "waypoint/max_vertical_velocity";
constexpr char kWaypointMaxVerticalAccelParam[] = "waypoint/max_vertical_accel";
constexpr char kWaypointMaxVerticalJerkParam[] = "waypoint/max_vertical_jerk";
constexpr char kWaypointMaxHeadingRateParam[] = "waypoint/max_heading_rate";
constexpr char kWaypointMaxHeadingAccelParam[] = "waypoint/max_heading_accel";
constexpr char kTakeoffMaxSpeedParam[] = "takeoff/max_speed";
constexpr char kTakeoffMaxAccelParam[] = "takeoff/max_accel";
constexpr char kTakeoffMaxJerkParam[] = "takeoff/max_jerk";
constexpr char kLandSpeedParam[] = "land/speed";
constexpr char kRtlMinAltitudeParam[] = "rtl/min_altitude";
}  // namespace

MulticopterWidget::MulticopterWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_XY_CRUISE
  wp_max_hor_vel_ = new qt::DoubleSpinBox();
  wp_max_hor_vel_->setDecimals(1);
  wp_max_hor_vel_->setMinimum(1.0);  // 3 m/s -> 1 m/s
  wp_max_hor_vel_->setMaximum(20.0);
  wp_max_hor_vel_->setSuffix(" m/s");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_HOR_MAX
  wp_max_hor_acc_ = new qt::DoubleSpinBox();
  wp_max_hor_acc_->setDecimals(1);
  wp_max_hor_acc_->setMinimum(2.0);
  wp_max_hor_acc_->setMaximum(15.0);
  wp_max_hor_acc_->setSuffix(" m/s²");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_JERK_AUTO
  wp_max_hor_jerk_ = new qt::DoubleSpinBox();
  wp_max_hor_jerk_->setDecimals(1);
  wp_max_hor_jerk_->setMinimum(1.0);
  wp_max_hor_jerk_->setMaximum(80.0);
  wp_max_hor_jerk_->setSuffix(" m/s³");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_Z_V_AUTO_DN
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_Z_V_AUTO_UP
  // TODO: Use separate settings for ascent and descent.
  wp_max_ver_vel_ = new qt::DoubleSpinBox();
  wp_max_ver_vel_->setDecimals(1);
  wp_max_ver_vel_->setMinimum(0.5);
  wp_max_ver_vel_->setMaximum(4.0);
  wp_max_ver_vel_->setSuffix(" m/s");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_DOWN_MAX
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_UP_MAX
  // TODO: Use separate settings for ascent and descent.
  wp_max_ver_acc_ = new qt::DoubleSpinBox();
  wp_max_ver_acc_->setDecimals(1);
  wp_max_ver_acc_->setMinimum(2.0);
  wp_max_ver_acc_->setMaximum(15.0);
  wp_max_ver_acc_->setSuffix(" m/s²");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_JERK_AUTO
  wp_max_ver_jerk_ = new qt::DoubleSpinBox();
  wp_max_ver_jerk_->setDecimals(1);
  wp_max_ver_jerk_->setMinimum(1.0);
  wp_max_ver_jerk_->setMaximum(80.0);
  wp_max_ver_jerk_->setSuffix(" m/s³");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_YAWRAUTO_MAX
  wp_max_head_rate_ = new qt::SpinBox();
  wp_max_head_rate_->setMinimum(5);
  wp_max_head_rate_->setMaximum(360);
  wp_max_head_rate_->setSuffix(" deg/s");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_YAWRAUTO_ACC
  wp_max_head_acc_ = new qt::SpinBox();
  wp_max_head_acc_->setMinimum(5);
  wp_max_head_acc_->setMaximum(360);
  wp_max_head_acc_->setSuffix(" deg/s²");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_TKO_SPEED
  takeoff_max_speed_ = new qt::DoubleSpinBox();
  takeoff_max_speed_->setDecimals(1);
  takeoff_max_speed_->setMinimum(1.0);
  takeoff_max_speed_->setMaximum(5.0);
  takeoff_max_speed_->setSuffix(" m/s");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_UP_MAX
  takeoff_max_accel_ = new qt::DoubleSpinBox();
  takeoff_max_accel_->setDecimals(1);
  takeoff_max_accel_->setMinimum(1.0);
  takeoff_max_accel_->setMaximum(15.0);
  takeoff_max_accel_->setSuffix(" m/s²");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_JERK_AUTO
  takeoff_max_jerk_ = new qt::DoubleSpinBox();
  takeoff_max_jerk_->setDecimals(1);
  takeoff_max_jerk_->setMinimum(1.0);
  takeoff_max_jerk_->setMaximum(80.0);
  takeoff_max_jerk_->setSuffix(" m/s³");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_LAND_SPEED
  land_speed_ = new qt::DoubleSpinBox();
  land_speed_->setDecimals(1);
  land_speed_->setMinimum(0.6);
  land_speed_->setMaximum(2.0);
  land_speed_->setSuffix(" m/s");

  // https://docs.px4.io/main/en/advanced_config/parameter_reference#RTL_RETURN_ALT
  rtl_min_alt_ = new qt::DoubleSpinBox();
  rtl_min_alt_->setDecimals(2);
  rtl_min_alt_->setMinimum(0.0);
  rtl_min_alt_->setMaximum(150.0);  // Japanese prohibited airspace.
  rtl_min_alt_->setSuffix(" m");

  const auto wp_form = new qt::FormLayout();
  wp_form->addVAlignedRow("Maximum Horizontal Velocity", wp_max_hor_vel_);
  wp_form->addVAlignedRow("Maximum Horizontal Acceleration", wp_max_hor_acc_);
  wp_form->addVAlignedRow("Maximum Horizontal Jerk", wp_max_hor_jerk_);
  wp_form->addVAlignedRow("Maximum Vertical Velocity", wp_max_ver_vel_);
  wp_form->addVAlignedRow("Maximum Vertical Acceleration", wp_max_ver_acc_);
  wp_form->addVAlignedRow("Maximum Vertical Jerk", wp_max_ver_jerk_);
  wp_form->addVAlignedRow("Maximum Heading Rate", wp_max_head_rate_);
  wp_form->addVAlignedRow("Maximum Heading Acceleration", wp_max_head_acc_);

  const auto takeoff_form = new qt::FormLayout();
  takeoff_form->addVAlignedRow("Maximum Speed", takeoff_max_speed_);
  takeoff_form->addVAlignedRow("Maximum Acceleration", takeoff_max_accel_);
  takeoff_form->addVAlignedRow("Maximum Jerk", takeoff_max_jerk_);

  const auto land_form = new qt::FormLayout();
  land_form->addVAlignedRow("Speed", land_speed_);

  const auto rtl_form = new qt::FormLayout();
  rtl_form->addVAlignedRow("Minimum Altitude (wrt. Launch Point)", rtl_min_alt_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("Waypoint", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(wp_form);
  rows->addWidget(new qt::Label("Takeoff", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(takeoff_form);
  rows->addWidget(new qt::Label("Land", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(land_form);
  rows->addWidget(new qt::Label("Return-to-Launch", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(rtl_form);

  setLayout(rows);
}

void MulticopterWidget::setToDefaults()
{
  wp_max_hor_vel_->setValue(5.0);
  wp_max_hor_acc_->setValue(5.0);
  wp_max_hor_jerk_->setValue(4.0);
  wp_max_ver_vel_->setValue(1.5);
  wp_max_ver_acc_->setValue(3.0);
  wp_max_ver_jerk_->setValue(4.0);
  wp_max_head_rate_->setValue(60);
  wp_max_head_acc_->setValue(120);  // 20 deg/s^2 -> 120 deg/s^2

  takeoff_max_speed_->setValue(1.5);
  takeoff_max_accel_->setValue(4.0);
  takeoff_max_jerk_->setValue(4.0);

  land_speed_->setValue(0.7);

  rtl_min_alt_->setValue(15.0);  // https://ardupilot.org/copter/docs/rtl-mode.html
}

QString MulticopterWidget::executorPackage() const
{
  return "tobas_mission_execution_mc";
}

QString MulticopterWidget::pluginName() const
{
  return "tobas::mission::MulticopterMissionExecutorNode";
}

YAML::Node MulticopterWidget::staticParams() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kWaypointMaxHorizontalVelocityParam] = yaml::format(wp_max_hor_vel_->value());
  node[kWaypointMaxHorizontalAccelParam] = yaml::format(wp_max_hor_acc_->value());
  node[kWaypointMaxHorizontalJerkParam] = yaml::format(wp_max_hor_jerk_->value());
  node[kWaypointMaxVerticalVelocityParam] = yaml::format(wp_max_ver_vel_->value());
  node[kWaypointMaxVerticalAccelParam] = yaml::format(wp_max_ver_acc_->value());
  node[kWaypointMaxVerticalJerkParam] = yaml::format(wp_max_ver_jerk_->value());
  node[kWaypointMaxHeadingRateParam] = yaml::format(st::deg2rad(wp_max_head_rate_->value()));
  node[kWaypointMaxHeadingAccelParam] = yaml::format(st::deg2rad(wp_max_head_acc_->value()));

  node[kTakeoffMaxSpeedParam] = yaml::format(takeoff_max_speed_->value());
  node[kTakeoffMaxAccelParam] = yaml::format(takeoff_max_accel_->value());
  node[kTakeoffMaxJerkParam] = yaml::format(takeoff_max_jerk_->value());

  node[kLandSpeedParam] = yaml::format(land_speed_->value());

  node[kRtlMinAltitudeParam] = yaml::format(rtl_min_alt_->value());

  return node;
}

YAML::Node MulticopterWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kWaypointMaxHorizontalVelocityParam] = yaml::format(wp_max_hor_vel_->value());
  node[kWaypointMaxHorizontalAccelParam] = yaml::format(wp_max_hor_acc_->value());
  node[kWaypointMaxHorizontalJerkParam] = yaml::format(wp_max_hor_jerk_->value());
  node[kWaypointMaxVerticalVelocityParam] = yaml::format(wp_max_ver_vel_->value());
  node[kWaypointMaxVerticalAccelParam] = yaml::format(wp_max_ver_acc_->value());
  node[kWaypointMaxVerticalJerkParam] = yaml::format(wp_max_ver_jerk_->value());
  node[kWaypointMaxHeadingRateParam] = wp_max_head_rate_->value();
  node[kWaypointMaxHeadingAccelParam] = wp_max_head_acc_->value();

  node[kTakeoffMaxSpeedParam] = yaml::format(takeoff_max_speed_->value());
  node[kTakeoffMaxAccelParam] = yaml::format(takeoff_max_accel_->value());
  node[kTakeoffMaxJerkParam] = yaml::format(takeoff_max_jerk_->value());

  node[kLandSpeedParam] = yaml::format(land_speed_->value());

  node[kRtlMinAltitudeParam] = yaml::format(rtl_min_alt_->value());

  return node;
}

void MulticopterWidget::load(const YAML::Node& node)
{
  wp_max_hor_vel_->setValue(node[kWaypointMaxHorizontalVelocityParam].as<double>());
  wp_max_hor_acc_->setValue(node[kWaypointMaxHorizontalAccelParam].as<double>());
  wp_max_hor_jerk_->setValue(node[kWaypointMaxHorizontalJerkParam].as<double>());
  wp_max_ver_vel_->setValue(node[kWaypointMaxVerticalVelocityParam].as<double>());
  wp_max_ver_acc_->setValue(node[kWaypointMaxVerticalAccelParam].as<double>());
  wp_max_ver_jerk_->setValue(node[kWaypointMaxVerticalJerkParam].as<double>());
  wp_max_head_rate_->setValue(node[kWaypointMaxHeadingRateParam].as<int>());
  wp_max_head_acc_->setValue(node[kWaypointMaxHeadingAccelParam].as<int>());

  takeoff_max_speed_->setValue(node[kTakeoffMaxSpeedParam].as<double>());
  takeoff_max_accel_->setValue(node[kTakeoffMaxAccelParam].as<double>());
  takeoff_max_jerk_->setValue(node[kTakeoffMaxJerkParam].as<double>());

  land_speed_->setValue(node[kLandSpeedParam].as<double>());

  rtl_min_alt_->setValue(node[kRtlMinAltitudeParam].as<double>());
}

bool MulticopterWidget::isValid()
{
  return true;
}
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
