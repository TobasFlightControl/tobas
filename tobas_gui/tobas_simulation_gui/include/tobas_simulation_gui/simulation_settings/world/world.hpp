#pragma once

#include <rclcpp/node.hpp>

#include "./base.hpp"

namespace gui
{
namespace sim
{
class WorldWidget : public QWidget
{
  Q_OBJECT

  using self = WorldWidget;
  using super = QWidget;

public:
  explicit WorldWidget(rclcpp::Node::SharedPtr node);

  std::filesystem::path worldPath() const;

private:
  std::vector<WorldWidget_Base*> widgets_;

  const WorldWidget_Base* selected() const;
};
}  // namespace sim
}  // namespace gui
