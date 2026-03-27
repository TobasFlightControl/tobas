#include "tobas_simulation_gui/simulation_settings/simulation_settings.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sim
{
SimulationSettingsWidget::SimulationSettingsWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Simulation Settings", cmn::kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  const auto scroll_rows = qt::createScrollableQVBoxLayout(rows);

  type_ = new LoopTypeWidget();
  scroll_rows->addWidget(type_);

  world_ = new WorldWidget(node);
  scroll_rows->addWidget(world_);

  pose_ = new PoseWidget();
  scroll_rows->addWidget(pose_);

  sbus_ = new SbusWidget();
  scroll_rows->addWidget(sbus_);

  debug_ = new DebugWidget();
  scroll_rows->addWidget(debug_);

  scroll_rows->addStretch();
}

LoopType SimulationSettingsWidget::loopType() const
{
  return type_->loopType();
}

fs::path SimulationSettingsWidget::worldPath() const
{
  return world_->worldPath();
}

double SimulationSettingsWidget::x() const
{
  return pose_->x();
}

double SimulationSettingsWidget::y() const
{
  return pose_->y();
}

double SimulationSettingsWidget::z() const
{
  return pose_->z();
}

double SimulationSettingsWidget::roll() const
{
  return pose_->roll();
}

double SimulationSettingsWidget::pitch() const
{
  return pose_->pitch();
}

double SimulationSettingsWidget::yaw() const
{
  return pose_->yaw();
}

fs::path SimulationSettingsWidget::sbusDevicePath() const
{
  return sbus_->devicePath();
}

bool SimulationSettingsWidget::userDebug() const
{
  return debug_->userDebug();
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
