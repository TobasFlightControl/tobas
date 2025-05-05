#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./dynamics/dynamics.hpp"
#include "./hardware_interface.hpp"
#include "./limit.hpp"
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
  static constexpr char kLimitLabel[] = "Limit";
  static constexpr char kHardwareIfaceLabel[] = "HW Interface";

public:
  explicit EngineWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const EngineDynamicsWidget* dynamics() const;
  const EngineResponseWidget* response() const;
  const EngineLimitWidget* limit() const;
  const EngineHardwareIfaceWidget* hardwareIface() const;

private:
  EngineDynamicsWidget* dynamics_;
  EngineResponseWidget* response_;
  EngineLimitWidget* limit_;
  EngineHardwareIfaceWidget* hw_iface_;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
