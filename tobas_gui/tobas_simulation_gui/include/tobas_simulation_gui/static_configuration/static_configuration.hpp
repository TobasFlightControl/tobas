#pragma once

#include "./simulation_type.hpp"
#include "./world/world.hpp"

namespace gui
{
namespace sim
{
class StaticConfigWidget : public QWidget
{
  Q_OBJECT

  using self = StaticConfigWidget;
  using super = QWidget;

public:
  explicit StaticConfigWidget(rclcpp::Node::SharedPtr node);

  sim_type_t simulationType() const;
  std::filesystem::path worldPath() const;

private:
  SimulationTypeWidget* type_;
  WorldWidget* world_;
};
}  // namespace sim
}  // namespace gui
