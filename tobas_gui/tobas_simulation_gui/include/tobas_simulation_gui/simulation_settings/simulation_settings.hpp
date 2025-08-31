#pragma once

#include "./debug.hpp"
#include "./loop_type.hpp"
#include "./world/world.hpp"

namespace gui
{
namespace sim
{
class SimulationSettingsWidget : public QWidget
{
  Q_OBJECT

  using self = SimulationSettingsWidget;
  using super = QWidget;

public:
  explicit SimulationSettingsWidget(rclcpp::Node::SharedPtr node);

  LoopType loopType() const;

  std::filesystem::path worldPath() const;

  bool userDebug() const;

private:
  LoopTypeWidget* type_;
  WorldWidget* world_;
  DebugWidget* debug_;
};
}  // namespace sim
}  // namespace gui
