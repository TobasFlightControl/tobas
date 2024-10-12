#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rcin_viewer/toggles_viewer.hpp"

namespace gui
{
namespace control_system
{
namespace rcin
{
TogglesViewer::TogglesViewer(rclcpp::Node::SharedPtr node) : node_(node)
{
  // テキストの長さを揃える
  program_mode_ = new qt::CircleWidget(" Program ");
  stabilize_mode_ = new qt::CircleWidget("Stabilize");
  acrobat_mode_ = new qt::CircleWidget(" Acrobat ");

  estop_ = new qt::ToggleSwitch();
  estop_->setText("E-Stop");
  estop_->ignoreMousePressEvent(true);

  gpsw_ = new qt::ToggleSwitch();
  gpsw_->setText(" GPSw ");
  gpsw_->ignoreMousePressEvent(true);

  // Layout
  const auto mode_cols = new QHBoxLayout();
  mode_cols->addWidget(program_mode_);
  mode_cols->addWidget(stabilize_mode_);
  mode_cols->addWidget(acrobat_mode_);

  const auto toggle_cols = new QHBoxLayout();
  toggle_cols->addWidget(estop_);
  toggle_cols->addWidget(gpsw_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(mode_cols, 3);
  rows->addLayout(toggle_cols, 2);

  setLayout(rows);

  reset();
}

void TogglesViewer::updateNamespace(const std::string& ns)
{
  reset();
  rcin_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kRcInputTopic), &self::rcInputCb, this);
}

void TogglesViewer::reset()
{
  stabilize_mode_->setColor(Qt::gray);
  acrobat_mode_->setColor(Qt::gray);
  program_mode_->setColor(Qt::gray);

  estop_->setChecked(false);
  gpsw_->setChecked(false);
}

void TogglesViewer::rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin)
{
  if (rcin->mode == tobas::flight_mode_t::PROGRAM_MODE)
    program_mode_->setColor(Qt::green);
  else
    program_mode_->setColor(Qt::gray);

  if (rcin->mode == tobas::flight_mode_t::STABILIZE_MODE)
    stabilize_mode_->setColor(Qt::green);
  else
    stabilize_mode_->setColor(Qt::gray);

  if (rcin->mode == tobas::flight_mode_t::ACROBAT_MODE)
    acrobat_mode_->setColor(Qt::green);
  else
    acrobat_mode_->setColor(Qt::gray);

  if (rcin->e_stop)
    estop_->setChecked(true);
  else
    estop_->setChecked(false);

  if (rcin->gpsw)
    gpsw_->setChecked(true);
  else
    gpsw_->setChecked(false);
}
}  // namespace rcin
}  // namespace control_system
}  // namespace gui
