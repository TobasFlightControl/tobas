#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rcin_viewer/toggles_viewer.hpp"

namespace gui
{
namespace gcs
{
namespace rcin
{
TogglesViewer::TogglesViewer(rclcpp::Node::SharedPtr node) : node_(node)
{
  acrobat_mode_ = new qt::CircleWidget(" Acrobat ");
  stabilize_mode_ = new qt::CircleWidget("Stabilize");
  loiter_mode_ = new qt::CircleWidget("  Loiter  ");

  enable_ = new qt::ToggleSwitch();
  enable_->setText("Enable");
  enable_->ignoreMousePressEvent(true);

  gpsw_ = new qt::ToggleSwitch();
  gpsw_->setText(" GPSw ");
  gpsw_->ignoreMousePressEvent(true);

  // Layout
  const auto mode_cols = new QHBoxLayout();
  mode_cols->addWidget(acrobat_mode_);
  mode_cols->addWidget(stabilize_mode_);
  mode_cols->addWidget(loiter_mode_);

  const auto toggle_cols = new QHBoxLayout();
  toggle_cols->addWidget(enable_);
  toggle_cols->addWidget(gpsw_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(toggle_cols, 2);
  rows->addLayout(mode_cols, 3);

  setLayout(rows);
}

void TogglesViewer::reset()
{
  acrobat_mode_->setColor(kOffColor);
  stabilize_mode_->setColor(kOffColor);
  loiter_mode_->setColor(kOffColor);

  enable_->setChecked(false);
  gpsw_->setChecked(false);
}

void TogglesViewer::updateNamespace(const std::string& ns)
{
  reset();

  rcin_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kRcInputTopic), &self::rcInputCb, this);
}

void TogglesViewer::rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin)
{
  enable_->setChecked(rcin->enable);
  gpsw_->setChecked(rcin->gpsw);

  const auto mode = static_cast<tobas::flight_mode_t>(rcin->mode);

  if (rcin->enable)
  {
    if (mode == tobas::flight_mode_t::STABILIZE)
      stabilize_mode_->setColor(kOnColorEnable);
    else
      stabilize_mode_->setColor(kOffColor);

    if (mode == tobas::flight_mode_t::ACROBAT)
      acrobat_mode_->setColor(kOnColorEnable);
    else
      acrobat_mode_->setColor(kOffColor);

    if (mode == tobas::flight_mode_t::LOITER)
      loiter_mode_->setColor(kOnColorEnable);
    else
      loiter_mode_->setColor(kOffColor);

    enable_->setOnColor(kOnColorEnable);
    gpsw_->setOnColor(kOnColorEnable);
  }
  else
  {
    if (mode == tobas::flight_mode_t::STABILIZE)
      stabilize_mode_->setColor(kOnColorDisable);
    else
      stabilize_mode_->setColor(kOffColor);

    if (mode == tobas::flight_mode_t::ACROBAT)
      acrobat_mode_->setColor(kOnColorDisable);
    else
      acrobat_mode_->setColor(kOffColor);

    if (mode == tobas::flight_mode_t::LOITER)
      loiter_mode_->setColor(kOnColorDisable);
    else
      loiter_mode_->setColor(kOffColor);

    enable_->setOnColor(kOnColorDisable);
    gpsw_->setOnColor(kOnColorDisable);
  }
}
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
