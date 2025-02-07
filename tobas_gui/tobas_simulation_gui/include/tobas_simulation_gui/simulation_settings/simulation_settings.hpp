#pragma once

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

  loop_type_t loopType() const;
  std::filesystem::path worldPath() const;

private:
  LoopTypeWidget* type_;
  WorldWidget* world_;
};
}  // namespace sim
}  // namespace gui
