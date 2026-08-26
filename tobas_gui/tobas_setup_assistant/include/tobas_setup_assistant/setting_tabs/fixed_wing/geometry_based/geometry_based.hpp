#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_uadf/model.hpp>

#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/calculator.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/control_surfaces.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/wings.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/manual/manual.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace gb
{
class GeometryBasedWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = GeometryBasedWidget;
  using super = qt::ScrollArea;

  static constexpr char kWingsLabel[] = "Wings";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";

public:
  explicit GeometryBasedWidget(const uadf::Model& uadf, const kdl::Tree& tree, mn::ManualWidget* manual);

  const char* name() const;
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  Calculator* calculator();

private:
  WingsWidget* wings_;
  ControlSurfacesWidget* control_surfaces_;
  Calculator* calculator_;
};
}  // namespace gb
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
