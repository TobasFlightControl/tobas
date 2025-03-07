#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

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

  kill_ = new qt::ToggleSwitch();
  sub_mode_ = new qt::ToggleSwitch();
  gpsw1_ = new qt::ToggleSwitch();
  gpsw2_ = new qt::ToggleSwitch();

  kill_->setText("  Kill  ");
  sub_mode_->setText("Sub Mode");
  gpsw1_->setText(" GPSw 1 ");
  gpsw2_->setText(" GPSw 2 ");

  kill_->ignoreMousePressEvent(true);
  sub_mode_->ignoreMousePressEvent(true);
  gpsw1_->ignoreMousePressEvent(true);
  gpsw2_->ignoreMousePressEvent(true);

  // Layout
  const auto toggle_cols = new QGridLayout();
  toggle_cols->addWidget(kill_, 0, 0);
  toggle_cols->addWidget(sub_mode_, 0, 1);
  toggle_cols->addWidget(gpsw1_, 1, 0);
  toggle_cols->addWidget(gpsw2_, 1, 1);

  const auto mode_cols = new QHBoxLayout();
  mode_cols->addWidget(acrobat_mode_);
  mode_cols->addWidget(stabilize_mode_);
  mode_cols->addWidget(loiter_mode_);

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

  kill_->setChecked(false);
  sub_mode_->setChecked(false);
  gpsw1_->setChecked(false);
  gpsw2_->setChecked(false);
}

void TogglesViewer::updateNamespace(const std::string& ns)
{
  reset();

  rcin_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kRcInputTopic), &self::rcInputCb, this);
}

void TogglesViewer::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  kill_->setChecked(rcin->kill);
  sub_mode_->setChecked(rcin->sub_mode);
  gpsw1_->setChecked(rcin->gpsw1);
  gpsw2_->setChecked(rcin->gpsw2);

  if (rcin->enable)
  {
    if (rcin->mode == tobas::flight_mode_t::STABILIZE)
      stabilize_mode_->setColor(kOnColorEnable);
    else
      stabilize_mode_->setColor(kOffColor);

    if (rcin->mode == tobas::flight_mode_t::ACROBAT)
      acrobat_mode_->setColor(kOnColorEnable);
    else
      acrobat_mode_->setColor(kOffColor);

    if (rcin->mode == tobas::flight_mode_t::LOITER)
      loiter_mode_->setColor(kOnColorEnable);
    else
      loiter_mode_->setColor(kOffColor);

    kill_->setOnColor(kOnColorEnable);
    sub_mode_->setOnColor(kOnColorEnable);
    gpsw1_->setOnColor(kOnColorEnable);
    gpsw2_->setOnColor(kOnColorEnable);
  }
  else
  {
    if (rcin->mode == tobas::flight_mode_t::STABILIZE)
      stabilize_mode_->setColor(kOnColorDisable);
    else
      stabilize_mode_->setColor(kOffColor);

    if (rcin->mode == tobas::flight_mode_t::ACROBAT)
      acrobat_mode_->setColor(kOnColorDisable);
    else
      acrobat_mode_->setColor(kOffColor);

    if (rcin->mode == tobas::flight_mode_t::LOITER)
      loiter_mode_->setColor(kOnColorDisable);
    else
      loiter_mode_->setColor(kOffColor);

    kill_->setOnColor(kOnColorDisable);
    sub_mode_->setOnColor(kOnColorDisable);
    gpsw1_->setOnColor(kOnColorDisable);
    gpsw2_->setOnColor(kOnColorDisable);
  }
}
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
