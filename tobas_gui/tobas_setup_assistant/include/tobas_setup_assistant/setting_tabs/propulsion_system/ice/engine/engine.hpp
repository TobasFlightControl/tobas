#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./dynamics.hpp"
#include "./response.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = EngineWidget;
  using super = qt::TabWidget;

  static constexpr int kTabWidth = 135;
  static constexpr int kTabHeight = 45;

  static constexpr char kDynamicsLabel[] = "Dynamics";
  static constexpr char kResponseLabel[] = "Response";

public:
  explicit EngineWidget(rclcpp::Node::SharedPtr node);

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const EngineDynamicsWidget* dynamics() const;
  const EngineResponseWidget* response() const;

private:
  EngineDynamicsWidget* dynamics_;
  EngineResponseWidget* response_;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
