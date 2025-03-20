#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./response.hpp"
#include "./dynamics/dynamics.hpp"

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

  static constexpr char kResponseLabel[] = "Response";
  static constexpr char kDynamicsLabel[] = "Dynamics";

public:
  explicit EngineWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const EngineResponseWidget* response() const;
  const EngineDynamicsWidget* dynamics() const;

private:
  EngineResponseWidget* response_;
  EngineDynamicsWidget* dynamics_;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
